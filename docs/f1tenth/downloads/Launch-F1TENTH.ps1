param([ValidateSet('Start','Stop','Export')][string]$Action='Start')
$ErrorActionPreference='Stop'
$root=$PSScriptRoot
$encoded=[Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($root))
$driveRoot=[IO.Path]::GetPathRoot($root)
$drive=''
$relative=''
if($driveRoot -match '^[A-Za-z]:\\$'){
 $drive=[Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($driveRoot))
 $relative=[Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($root.Substring($driveRoot.Length).Replace('\','/')))
}
$option=if($Action -eq 'Stop'){'--stop'}elseif($Action -eq 'Export'){'--export-kit'}else{''}
$script=@"
set -euo pipefail
# Translate the drive root, then append the relative path. Translating an exact
# Docker-mounted subfolder can instead return a Docker Desktop mount alias whose
# parent is not the actual Windows parent (notably for export destinations).
if [ -n '$drive' ]; then
 base=`$(wslpath -u "`$(printf '%s' '$drive' | base64 -d)")
 root="`${base%/}/`$(printf '%s' '$relative' | base64 -d)"
else
 root=`$(wslpath -u "`$(printf '%s' '$encoded' | base64 -d)")
fi
exec python3 "`$root/F1TENTH_RACER.py" --root "`$root" $option
"@
$payload=[Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($script.Replace("`r",'')))
Write-Host 'F1TENTH Gym sandbox only. This does not launch/stop the AutoDRIVE controller.' -ForegroundColor Cyan
wsl.exe -d Ubuntu -- bash -lc "source <(echo $payload | base64 -d)"
if($LASTEXITCODE -ne 0){throw 'F1TENTH launcher failed; see the error above and F1TENTH_Sim\logs.'}
