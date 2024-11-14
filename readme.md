# F1Tenth Workspace


## Using via Dev Containers
- start by installing WSL: `wsl --install`
- install [Docker Destkop](https://www.docker.com/products/docker-desktop/): `winget install Docker.DockerDesktop` (needs reboot after install + setup in GUI)
- install [Dev Containers](vscode:extension/ms-vscode-remote.remote-containers) extension in VS Code
- if you already have this folder opened in code then `Ctrl + Shift + P` and `Dev Containers: Reopen folder in container` or `Dev Containers: Open Folder in Container`
- once container is built test running project with `Ctrl + Shift + B`


## Poznámky
### Obecné
- Při rozjezdu na začátku omezit zatáčení (rozjíždět se jenom rovně)
- Není synchronizovaná mapa v simulátoru s launch.py

### Samplování
- Používání klotoid

### Simulace controls
- Zkusit více modelů

### Evaluace
- Když se ohodnocuje jenom koncový stav, nezachytí se kolize na trajektorii
- Costmapa:
    - Lepší evaluace odměny za ujetou vzdálenost od ega
    - zastavit simulaci, než se načte costmapa
- Pipeline - nejdrive vyhodit kolize, potom resit zbytek
- Rozdelit opponent/race score
- Vyladit jezdeni bez oponenta