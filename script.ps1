# Variables
$User = "robot"
$Robot = "169.254.89.16"
$Password = "maker"
$RemoteDir = "/home/robot/competicion"

# Eliminar el directorio remoto
Write-Host "Eliminando el directorio remoto..."
ssh "${User}@${Robot}" "rm -rf ${RemoteDir}"

# Crear el directorio remoto
Write-Host "Creando el directorio remoto..."
ssh "${User}@${Robot}" "mkdir -p ${RemoteDir}"

# Copiar los archivos al directorio remoto
Write-Host "Copiando archivos al directorio remoto..."
scp movement.py reactive.py main.py "${User}@${Robot}:${RemoteDir}"

Write-Host "Despliegue completado."
