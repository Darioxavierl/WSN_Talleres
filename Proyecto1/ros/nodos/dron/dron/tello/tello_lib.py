import socket
import threading
import time
import av
import cv2
import numpy as np
from queue import Queue, Full

class Tello:
    def __init__(self, ssid="TELLO-636ECF", ip="192.168.10.1", port=8889, state_port=8890, video_port=11111, timeout=10):
        self.address = (ip, port)
        self.state_port = state_port
        self.cmd_port = port
        self.video_port = video_port
        self.sock_timeout = timeout
        self.ssid = ssid

        self.sock_cmd = None
        self.sock_state = None
        self.sock_video = None

        self._listening = True
        self._watching = False
        self._sdk_mode = False

        self._frame_callback = None
        self._stop_event = threading.Event()

    def _init_command_socket(self):
        """
        Inicializa el socket de comando con puerto efímero (0).
        Cada instancia de Tello tendrá su propio puerto local único.
        """
        try:
            self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # CLAVE: Bind a ('', 0) -> el SO asigna un puerto libre automáticamente
            self.sock_cmd.bind(('', 0))
            self.sock_cmd.settimeout(self.sock_timeout)
            local_port = self.sock_cmd.getsockname()[1]
            print(f"[+] Socket de comando inicializado en puerto local: {local_port}")
        except Exception as e:
            print(f"[!] Error al inicializar socket de comando: {e}")
            self.sock_cmd = None

    def conn_sock(self, sock_name, port, bind_socket=True):
        """Crea sockets temporales (solo para state y video)"""
        sock = getattr(self, sock_name)
        if sock is None or sock.fileno() == -1:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if bind_socket:
                sock.bind(('', port))
            sock.settimeout(self.sock_timeout)
            setattr(self, sock_name, sock)

    def close_sock(self, sock_name):
        """Cierra sockets temporales (NO el socket de comando)"""
        if sock_name == 'sock_cmd':
            return
            
        sock = getattr(self, sock_name)
        if sock:
            try:
                sock.close()
            except Exception as e:
                print(f"[!] Error al cerrar {sock_name}: {e}")
            setattr(self, sock_name, None)

    def _listen(self):
        """Escucha mensajes de estado del dron (uso temporal del socket)"""
        self.conn_sock('sock_state', self.state_port, bind_socket=True)
        try:
            data, _ = self.sock_state.recvfrom(1024)
            return data.decode('utf-8', errors='ignore').strip()
        except socket.timeout:
            return None
        except Exception as e:
            print(f"[!] Error en _listen(): {e}")
            return None
        finally:
            self.close_sock("sock_state")

    def _stream_video(self, state):
        """Activa/desactiva la transmisión de video"""
        if state == 1:
            print("[*] Iniciando video stream...")
            
            # PRIMERO: Asegurarse de que el stream esté apagado
            print("[*] Enviando streamoff por precaución...")
            resp = self.send_command("streamoff", timeout=2)
            print(f"[DEBUG] streamoff: {resp}")
            time.sleep(3)  # Esperar a que se detenga completamente

            print("[*] Enviando streamon...")
            resp = self.send_command("streamon", timeout=2)
            print(f"[DEBUG] streamon: {resp}")
            time.sleep(3)  # Esperar a que se detenga completamente

        elif state == 0:
            
            print("[*] Deteniendo video stream...")
            self._watching = False
            resp = self.send_command("streamoff", timeout=2)
            print(f"[DEBUG] streamoff previo: {resp}")
            time.sleep(1.5)  # Esperar a que se detenga completamente


    def start_video(self):
        """Lanza el hilo que recibe los frames H264 y los decodifica."""
        if self._watching:
            print("[!] Stream ya activo.")
            return
        # Ahora sí, iniciar el stream
        self._stop_event.clear()
        self._watching = True
        self._video_thread = threading.Thread(target=self._video_loop, daemon=True)
        self._video_thread.start()

    def stop_video(self):
        
        """Detiene el hilo de recepción de video."""
        if not self._watching:
            return
        print("[*] Deteniendo hilo de video...")
        self._stop_event.set()
        self._watching = False
        if self._video_thread and self._video_thread.is_alive():
            self._video_thread.join(timeout=2)
        print("[+] Hilo de video detenido.")

    def set_video_callback(self, callback):
        """callback: función que recibe un frame OpenCV cada vez que llega"""
        self._frame_callback = callback

    def _video_loop(self):
        """
        Recibe y decodifica el stream de video del dron usando OpenCV (FFmpeg backend).
        Llama a self._frame_callback(img) en cada frame recibido.
        """
        print(f"[*] Iniciando loop de video (UDP {self.video_port})...")

        # Abrir el stream de video con OpenCV
        cap = cv2.VideoCapture(f"udp://0.0.0.0:{self.video_port}", cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimizar la latencia

        if not cap.isOpened():
            print(f"[!] No se pudo abrir el stream de video en puerto {self.video_port}")
            return

        last_frame_time = time.time()

        while not self._stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                # Si no hay frames por un tiempo, asumimos que se perdió el stream
                if time.time() - last_frame_time > 3:
                    print("[!] Timeout de video — no se reciben frames.")
                    break
                continue

            last_frame_time = time.time()

            # Llamar callback (ROS publicará y mostrará el frame)
            if self._frame_callback:
                try:
                    self._frame_callback(frame)
                except Exception as e:
                    print(f"[!] Error en callback de frame: {e}")

        cap.release()
        print("[+] Loop de video finalizado correctamente.")




    def send_command(self, cmd, wait_response=True, timeout=None):
        """
        Envía un comando al dron usando el socket persistente.
        THREAD-SAFE: Cada instancia tiene su propio socket en puerto único.
        
        Args:
            cmd: Comando a enviar
            wait_response: Si False, no espera respuesta (fire-and-forget)
            timeout: Timeout personalizado para este comando (None usa el default)
        """
        if self.sock_cmd is None or self.sock_cmd.fileno() == -1:
            print("[!] Socket de comando no disponible, reinicializando...")
            self._init_command_socket()
            if self.sock_cmd is None:
                return None

        try:
            # Guardar timeout original si se especifica uno custom
            original_timeout = None
            if timeout is not None:
                original_timeout = self.sock_cmd.gettimeout()
                self.sock_cmd.settimeout(timeout)
            
            # Enviar comando
            self.sock_cmd.sendto(cmd.encode('utf-8'), self.address)
            
            # Si no queremos respuesta, retornar inmediatamente
            if not wait_response:
                return "sent"
            
            # Esperar respuesta
            data, server = self.sock_cmd.recvfrom(1024)
            response = data.decode('utf-8', errors='ignore').strip()
            
            # Restaurar timeout original
            if original_timeout is not None:
                self.sock_cmd.settimeout(original_timeout)
            
            # Pequeña pausa para no saturar el dron
            time.sleep(0.05)
            
            return response
            
        except socket.timeout:
            # Restaurar timeout original
            if timeout is not None and original_timeout is not None:
                self.sock_cmd.settimeout(original_timeout)
            print(f"[!] Timeout al enviar comando: {cmd}")
            return None
        except Exception as e:
            # Restaurar timeout original
            if timeout is not None and original_timeout is not None:
                self.sock_cmd.settimeout(original_timeout)
            print(f"[!] Error en send_command({cmd}): {e}")
            return None

    def enter_sdk_mode(self):
        """Entra en modo SDK"""
        resp = self.send_command("command")
        if resp and resp.lower() == "ok":
            self._sdk_mode = True
            print("[+] Modo SDK activado")
            return True
        print("[!] No se pudo entrar en modo SDK.")
        return False

    def get_battery(self):
        """Obtiene el nivel de batería"""
        return self.send_command("battery?")

    def takeoff(self):
        """Despega el dron"""
        return self.send_command("takeoff")

    def land(self):
        """Aterriza el dron"""
        return self.send_command("land")

    def move_forward(self, distance):
        """Mueve el dron hacia adelante (20-500 cm)"""
        return self.send_command(f"forward {distance}")

    def move_back(self, distance):
        """Mueve el dron hacia atrás (20-500 cm)"""
        return self.send_command(f"back {distance}")

    def move_left(self, distance):
        """Mueve el dron hacia la izquierda (20-500 cm)"""
        return self.send_command(f"left {distance}")

    def move_right(self, distance):
        """Mueve el dron hacia la derecha (20-500 cm)"""
        return self.send_command(f"right {distance}")

    def move_up(self, distance):
        """Mueve el dron hacia arriba (20-500 cm)"""
        return self.send_command(f"up {distance}")

    def move_down(self, distance):
        """Mueve el dron hacia abajo (20-500 cm)"""
        return self.send_command(f"down {distance}")

    def rotate_cw(self, degrees):
        """Rota el dron en sentido horario (1-360 grados)"""
        return self.send_command(f"cw {degrees}")

    def rotate_ccw(self, degrees):
        """Rota el dron en sentido antihorario (1-360 grados)"""
        return self.send_command(f"ccw {degrees}")

    def check_connection(self, tries=2):
        """
        Verifica si el dron responde al comando 'command' del SDK.
        Retorna True si responde 'ok', False en caso contrario.
        """
        for attempt in range(tries):
            resp = self.send_command("command")
            if resp and resp.lower() == "ok":
                return True
            if attempt < tries - 1:
                time.sleep(0.5)
        return False

    def close(self):
        """Cierra todos los sockets al finalizar"""
        self.stop_video()
        if self.sock_cmd:
            try:
                self.sock_cmd.close()
            except:
                pass
        if self.sock_state:
            try:
                self.sock_state.close()
            except:
                pass

    def __del__(self):
        """Destructor para limpiar recursos"""
        self.close()