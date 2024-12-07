# F1Tenth Workspace

## Struktura tříd

```mermaid
classDiagram
    AgentBase --|> PurePursuitAgent
    AgentBase --|> MapEvaluatingAgentBase
    MapEvaluatingAgentBase --|> SamplingAgent
    
    class AgentBase {
        string agent_namesapce: "ego_racecar"
        string opponent_namesapce: "opp_racecar"
        string state_topic: "state"
        string drive_topic: "drive"
        float velocity_gain: 1.0
        bool opponent_presend: False

        Subscriber(Float64MultiArray, agent_namesapce/state_topic)
        Subscriber(Float64MultiArray, opponent_namesapce/state_topic)
        Publisher(AckermannDriveStamped, agent_namesapce/drive_topic)
        Timer(30ms, update_control)
    }

    class MapEvaluatingAgentBase {
        string costmap_topic: "costmap"
        string map_name
        string map_folder_path

        Publisher(OccupancyGrid, agent_namesapce/costmap_topic)
        Timer(10ms, update_cotmap)

    }

    class SamplingAgent {
        string predictions_topic: "predictions"
        string followed_trajectory_topic: "followed_trajectory"

        Publisher(PointCloud2, agent_namesapce/predictions)
        Publisher(PointCloud2, agent_namesapce/followed_trajectory)

    }

    class PurePursuitAgent {
        string map_name
        string map_folder_path
    }
```

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
- zpomalit cas (vlastni publisher na cas), nebo nahrat rosbag a prehravat zpomalene

### Samplování
- Používání klotoid
- zlepsit limity pro generovani samplu

### Simulace controls
- Zkusit více modelů (Ackermann kinematic model)

### Evaluace
- Vyladit jezdeni bez oponenta
- pridat orientaci auta do vyhodnoceni
- napocitat vektory s orientaci
- odladit na centerline
