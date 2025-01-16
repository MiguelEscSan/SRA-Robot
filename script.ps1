# Variables
$User = "robot"
$Robot = "169.254.19.124"
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
scp movement.py main.py reactive.py "${User}@${Robot}:${RemoteDir}"

Write-Host "Despliegue completado."
