# F1Tenth Workspace


## Using via Dev Containers
- start by installing WSL: `wsl --install`
- install [Docker Destkop](https://www.docker.com/products/docker-desktop/): `winget install Docker.DockerDesktop` (needs reboot after install + setup in GUI)
- install [Dev Containers](vscode:extension/ms-vscode-remote.remote-containers) extension in VS Code
- if you already have this folder opened in code then `Ctrl + Shift + P` and `Dev Containers: Reopen folder in container` or `Dev Containers: Open Folder in Container`
- once container is built test running project: `New Terminal` and type in `ros2 launch launch.py`