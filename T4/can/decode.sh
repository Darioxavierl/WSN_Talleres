#!/bin/bash
# decode.sh - Decodifica mensajes CAN con delimitadores <...>
# Compatible con candump -L (salida tipo can0 200#3C5753...)
# Uso: ./decode.sh

echo "[+] Escuchando en can0 (ID 0x200)..."

buffer=""

# candump -L produce líneas como:
# (timestamp) can0 200#3C57534E20477275
candump -L can0,200:7FF | while IFS= read -r line; do
  # Muestra la línea cruda
  echo "[CAN] $line"

  # Extrae la parte hex tras el '#'
  hex=$(echo "$line" | awk -F'#' '{print $2}')
  # Si no hay parte hex, salta
  if [[ -z "$hex" ]]; then
    continue
  fi

  # Normalizar a mayúsculas (opcional)
  hex=${hex^^}

  # Recorre la cadena de hex de 2 en 2 caracteres (cada byte)
  len=${#hex}
  i=0
  while (( i < len )); do
    byte=${hex:i:2}
    # Si quedó un nibble suelto (improbable) lo ignora
    if [[ ${#byte} -ne 2 ]]; then
      break
    fi

    # Convierte el byte hex a carácter
    # printf "\\x$byte" funciona bien para valores ASCII
    c=$(printf "\\x$byte")

    # Debug opcional por byte (comenta si no quieres tanto output)
    printf "[BYTE] %s -> %s\n" "$byte" "$c"

    if [[ "$c" == "<" ]]; then
      buffer=""
      echo "[*] Inicio de mensaje detectado"
    elif [[ "$c" == ">" ]]; then
      echo "[✔] Mensaje completo recibido: $buffer"
      buffer=""
    else
      buffer+="$c"
    fi

    (( i += 2 ))
  done

done

