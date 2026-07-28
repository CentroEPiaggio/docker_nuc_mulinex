# Mulinex Social Robot — Demo 0

`socialbot_demo0` aggiunge al workspace Mulinex una demo Docker-first per
simulazione, mapping, navigazione e intervista audio. ROS 2 non deve essere
installato sul computer host.

La demo include:

- ROS 2 Humble nel container Docker;
- Gazebo Fortress / Ignition Gazebo;
- modello Mulinex `omnicar`;
- LiDAR 2D e camera RGB-D simulati;
- odometria ground-truth convertita nel frame `odom -> base_link`;
- SLAM Toolbox per creare una mappa;
- Nav2 con AMCL per navigare sulla mappa fornita;
- missione deterministica casa → intervista → casa;
- intervista reale oppure dialogo completamente preregistrato.

Non viene usato un LLM: Demo 0 verifica locomozione, sensori, navigazione,
audio e orchestrazione in modo deterministico.

## Requisiti dell'host

- Ubuntu con sessione grafica X11/XWayland;
- Docker Engine e Docker Compose v2;
- `git`, `xauth`, PulseAudio oppure PipeWire compatibile con PulseAudio;
- accesso Docker configurato senza `sudo`.

Il pacchetto è già integrato nel repository: non occorre installare o estrarre
un overlay separato.

### Robot NUC e laptop di sviluppo

`docker/compose.yaml` è la configurazione di base destinata al NUC del robot.
Sul NUC si usano normalmente i comandi Make senza opzioni aggiuntive.

Sul laptop, generare prima le impostazioni locali per X11 e audio:

```bash
bash demo0/setup_host.bash
```

Applicare quindi `compose.laptop.yaml` sopra la configurazione del NUC:

```bash
export COMPOSE_FILES="--env-file demo0/.env -f docker/compose.yaml -f docker/compose.laptop.yaml"
make build
make start
make demo0-build
make demo0-run
```

La variabile `COMPOSE_FILES` deve rimanere impostata anche per `make stop` e
per gli altri comandi della stessa sessione. L'override mantiene il servizio
`mulinex` e aggiunge soltanto impostazioni per Gazebo, Xauthority, audio host
e rendering software.

## Avvio rapido

Dalla root del repository:

```bash
make build
make start
make demo0-build
make demo0-run
```

Per arrestare il container:

```bash
make stop
```

## Comandi Make disponibili

| Comando | Funzione |
|---|---|
| `make demo0-build` | Valida e compila il workspace con `--symlink-install` |
| `make demo0-validate` | Esegue soltanto i controlli statici |
| `make demo0-shell` | Apre una shell nel container |
| `make demo0-audio` | Verifica la riproduzione audio |
| `make demo0-sim` | Avvia Gazebo, robot, sensori e RViz |
| `make demo0-mapping` | Avvia la simulazione con SLAM Toolbox |
| `make demo0-teleop` | Avvia la guida da tastiera |
| `make demo0-save-map` | Salva la mappa creata con SLAM |
| `make demo0-navigation` | Avvia Nav2 senza la missione |
| `make demo0-run` | Avvia la missione completa |
| `make demo0-run-no-audio` | Avvia la missione senza audio |
| `make demo0-scripted` | Avvia il dialogo completamente preregistrato |
| `make demo0-realsense` | Avvia una RealSense fisica |
| `make demo0-check` | Controlla i topic ROS 2 richiesti |

I target usano il servizio Compose `mulinex`. Per impostazione predefinita
caricano `docker/compose.yaml`; sul laptop usano anche l'override indicato
sopra.

## Simulazione minima

Avviare:

```bash
make demo0-sim
```

In un secondo terminale:

```bash
make demo0-check
```

Topic obbligatori:

```text
/clock
/scan
/odom
/joint_states
/omni_controller/twist_cmd
```

Topic camera attesi:

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw
```

La camera non è necessaria per la missione. Se i nomi differiscono, controllare
con `ign topic -l` e aggiornare il bridge in
`launch/simulation.launch.py`.

## Guida manuale

Nel primo terminale:

```bash
make demo0-sim
```

Nel secondo:

```bash
make demo0-teleop
```

Comandi principali di `teleop_twist_keyboard`:

```text
i  avanti
,  indietro
j  ruota a sinistra
l  ruota a destra
k  stop
```

Il nodo `cmd_vel_relay` limita e inoltra `/cmd_vel` a
`/omni_controller/twist_cmd`, continuando a inviare zero quando il comando
scade. La velocità laterale è disabilitata in questa prima versione.

## Creazione della mappa

Avviare SLAM e teleoperazione in due terminali:

```bash
make demo0-mapping
```

```bash
make demo0-teleop
```

Dopo avere esplorato lentamente la stanza:

```bash
make demo0-save-map
```

La mappa viene salvata in:

```text
src/socialbot_demo0/maps/my_demo0_map.pgm
src/socialbot_demo0/maps/my_demo0_map.yaml
```

La missione usa inizialmente `maps/demo0_map.yaml`.

## Nav2 senza missione

```bash
make demo0-navigation
```

In RViz verificare che mappa, scansione e robot coincidano. Usare `2D Pose
Estimate` se la posa iniziale non è stata impostata e `Nav2 Goal` per inviare
un obiettivo manuale.

La catena dei comandi è:

```text
Nav2 controller_server
  -> cmd_vel_nav
Nav2 velocity_smoother
  -> /cmd_vel
cmd_vel_relay
  -> /omni_controller/twist_cmd
omni_controller
  -> ruote simulate
```

## Missione completa

Per una persona reale che risponde durante le pause:

```bash
make demo0-run
```

La sequenza prevista è:

1. partenza dalla posa `home`;
2. inizializzazione automatica di AMCL;
3. navigazione verso `interview_spot`;
4. introduzione e prima domanda;
5. attesa della risposta;
6. seconda domanda e nuova attesa;
7. saluto;
8. ritorno a `home`.

Per testare navigazione e orchestrazione senza audio:

```bash
make demo0-run-no-audio
```

Per il dialogo completamente preregistrato:

```bash
make demo0-scripted
```

Missione, waypoint e sequenza audio sono configurati in:

```text
config/interview_script.yaml
config/interview_script_fully_scripted.yaml
```

Con `--symlink-install`, le modifiche ai file Python e YAML normalmente non
richiedono una nuova compilazione. Dopo modifiche a `setup.py`, `package.xml`
o all'elenco dei file installati, eseguire nuovamente `make demo0-build`.

## Audio e speaker Bluetooth

Il Bluetooth viene gestito dall'host, non dal container:

```bash
bluetoothctl
```

Comandi tipici:

```text
power on
agent on
default-agent
scan on
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
quit
```

Controllare e selezionare il sink:

```bash
pactl list short sinks
pactl set-default-sink NOME_DEL_SINK
make demo0-audio
```

Il socket PulseAudio/PipeWire dell'host deve essere montato nel container
tramite `docker/compose.yaml`.

Per rigenerare gli audio offline:

```bash
make demo0-shell
cd ~/mulinex_ws
./demo0/generate_audio.bash
exit
make demo0-build
```

## RealSense D435 fisica

Demo 0 usa normalmente la camera simulata. Dopo avere fermato i launch
simulativi, collegare la D435 a una porta USB 3 ed eseguire:

```bash
make demo0-realsense
```

Il passaggio completo di `/dev` al container è comodo nello sviluppo, ma per
il deployment è preferibile autorizzare soltanto i dispositivi necessari.

## Risoluzione dei problemi

### Gazebo o RViz non appaiono

Sull'host controllare:

```bash
echo "$DISPLAY"
ls -l /tmp/.docker.xauth
```

Per provare il rendering software:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
make demo0-sim
```

### Controller inattivo

Dentro `make demo0-shell`:

```bash
ros2 service call /omni_controller/activate_srv std_srvs/srv/SetBool "{data: true}"
```

Il nodo `activate_omni_controller` tenta già questa attivazione
automaticamente.

### Nav2 non muove il robot

```bash
ros2 topic hz /cmd_vel
ros2 topic hz /omni_controller/twist_cmd
```

### Topic `/scan` assente

```bash
ign topic -l | grep -i lidar
```

Se il topic Gazebo non è `/lidar`, aggiornare argomento e remapping in
`launch/simulation.launch.py`.

### Audio assente

Controllare `pactl info` e `pactl list short sinks` sia sull'host sia nel
container. Il sink Bluetooth deve essere raggiungibile attraverso il server
audio dell'host.

## Come creare una nuova demo

Una nuova demo deve essere aggiunta come pacchetto ROS 2 nel medesimo
workspace. Non deve avere un Dockerfile, un entrypoint o un Compose completo
separato. Il container e la configurazione del NUC rimangono condivisi.

Nell'esempio seguente la nuova demo si chiama `socialbot_demo1`.

### 1. Creare il pacchetto

Se la demo è un'evoluzione di Demo 0, copiarne la struttura senza file
generati:

```bash
rsync -a \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  src/socialbot_demo0/ \
  src/socialbot_demo1/
```

Rinominare quindi il modulo Python:

```bash
mv src/socialbot_demo1/socialbot_demo0 \
   src/socialbot_demo1/socialbot_demo1
```

Sostituire `socialbot_demo0` con `socialbot_demo1` almeno nei seguenti file:

```text
src/socialbot_demo1/package.xml
src/socialbot_demo1/setup.py
src/socialbot_demo1/setup.cfg
src/socialbot_demo1/resource/
src/socialbot_demo1/launch/*.launch.py
src/socialbot_demo1/socialbot_demo1/*.py
```

Anche il file marker in `resource/` deve essere rinominato:

```bash
mv src/socialbot_demo1/resource/socialbot_demo0 \
   src/socialbot_demo1/resource/socialbot_demo1
```

Controllare che in `setup.py` siano corretti:

- `package_name`;
- nome del pacchetto;
- moduli indicati in `console_scripts`;
- file installati da `config`, `maps`, `worlds` e `assets`;
- descrizione e versione.

Aggiornare in `package.xml` nome, descrizione, maintainer, licenza e dipendenze.

### 2. Definire lo scopo prima del launch completo

Scrivere una breve sequenza deterministica, per esempio:

```text
home -> waypoint -> azione -> home
```

Separare chiaramente:

- comportamento condiviso con Demo 0;
- nuovi nodi ROS 2;
- nuovi sensori o dispositivi;
- elementi esclusivamente simulativi;
- comportamento necessario sul robot reale.

Se più demo usano lo stesso nodo, è preferibile spostarlo in un pacchetto
condiviso invece di mantenerne copie divergenti.

### 3. Aggiungere configurazioni e risorse

Usare la struttura seguente:

```text
src/socialbot_demo1/
├── assets/       audio e altre risorse runtime
├── config/       missione, Nav2, SLAM e parametri
├── launch/       launch separati e launch completo
├── maps/         mappe PGM/YAML
├── resource/     marker ament
├── socialbot_demo1/
├── worlds/       mondi usati soltanto in simulazione
├── package.xml
├── setup.cfg
└── setup.py
```

I file runtime devono essere installati da `setup.py`; non basta che esistano
nel sorgente. Verificare in particolare mappe, mondi, YAML e audio.

Per la missione è consigliato mantenere waypoint e sequenza in YAML, lasciando
al nodo Python soltanto orchestrazione, gestione degli errori e chiamate ROS.

### 4. Creare launch modulari

Mantenere separati almeno:

```text
simulation.launch.py
mapping.launch.py
navigation.launch.py
demo1.launch.py
```

Il launch completo dovrebbe includere gli altri e aggiungere soltanto la
missione. I nodi esclusivamente simulativi, come odometria ground-truth e
bridge Gazebo, non devono essere avviati sul robot reale.

Esporre come argomenti launch almeno:

- `use_sim_time`;
- abilitazione di RViz;
- abilitazione dell'audio;
- file della missione;
- file della mappa;
- modalità simulazione o robot reale, quando necessaria.

### 5. Aggiornare Docker soltanto per nuove dipendenze

Non creare `DockerFile.demo1`. Aggiungere a `docker/DockerFile` soltanto i
pacchetti APT/ROS realmente nuovi.

Esempio:

```dockerfile
ros-$ROS_DISTRO-nome-pacchetto
```

Non modificare l'entrypoint per logica specifica della demo. L'entrypoint deve
continuare soltanto a preparare ROS, workspace e directory runtime.

### 6. Mantenere Compose comune

Non creare una copia completa del Compose per ogni demo:

- `docker/compose.yaml` resta la base del NUC;
- `docker/compose.laptop.yaml` resta l'override del laptop;
- una modifica Compose è necessaria soltanto per nuovi dispositivi, mount,
  variabili host o permessi.

Una configurazione specifica della missione appartiene normalmente in un file
YAML del pacchetto, non nel Compose.

### 7. Aggiungere i target Make

Seguire lo schema dei target Demo 0:

```make
demo1-build:
	$(DEMO0_EXEC) 'source /opt/ros/humble/setup.bash; cd $$HOME/mulinex_ws; colcon build --symlink-install --packages-up-to socialbot_demo1'

demo1-sim:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); ros2 launch socialbot_demo1 simulation.launch.py use_rviz:=true'

demo1-run:
	$(DEMO0_EXEC) '$(DEMO0_SHELL); ros2 launch socialbot_demo1 demo1.launch.py'
```

È possibile rinominare in futuro `DEMO0_EXEC` e `DEMO0_SHELL` in nomi generici,
per esempio `ROS_EXEC` e `ROS_SHELL`, quando vengono condivisi da più demo.
Ricordarsi di aggiungere i nuovi target a `.PHONY`.

### 8. Aggiungere la validazione

La validazione minima dovrebbe controllare:

- sintassi di Python, YAML, XML e SDF;
- presenza dei file installati;
- esistenza e formato delle risorse audio;
- coerenza fra mappa, posa iniziale e waypoint;
- dipendenze dichiarate in `package.xml`;
- assenza di riferimenti rimasti a `socialbot_demo0`.

Il validatore attuale è specifico di Demo 0. Per Demo 1 si può inizialmente
creare un validatore dedicato, poi estrarre le verifiche comuni in uno script
riutilizzabile quando esistono almeno due demo.

### 9. Compilare e verificare per livelli

Sul laptop:

```bash
bash demo0/setup_host.bash
export COMPOSE_FILES="--env-file demo0/.env -f docker/compose.yaml -f docker/compose.laptop.yaml"
make build
make start
```

Compilare prima soltanto il nuovo pacchetto e le sue dipendenze:

```bash
docker compose $COMPOSE_FILES exec mulinex bash -lc \
  'source /opt/ros/humble/setup.bash; cd $HOME/mulinex_ws; colcon build --symlink-install --packages-up-to socialbot_demo1'
```

Eseguire i test in quest'ordine:

1. import e scoperta del pacchetto;
2. simulazione, robot e sensori;
3. controllo dei topic;
4. teleoperazione;
5. mapping;
6. navigazione verso un goal manuale;
7. missione senza audio;
8. test audio;
9. missione completa;
10. prova sul NUC e infine sul robot reale.

Controllo minimo del pacchetto:

```bash
make demo0-shell
source /opt/ros/humble/setup.bash
source ~/mulinex_ws/install/setup.bash
ros2 pkg prefix socialbot_demo1
```

### 10. Preparare il passaggio al NUC

Prima di portare la demo sul robot:

- verificare che non dipenda da percorsi assoluti del laptop;
- non aggiungere `build/`, `install/`, `log/`, `.env`, `__pycache__` o `.pyc`;
- verificare dispositivi e permessi richiesti;
- distinguere odometria simulata e odometria reale;
- impostare limiti di velocità conservativi;
- prevedere timeout, stop e comportamento sicuro in caso di errore;
- provare il Compose base del NUC senza l'override laptop;
- documentare avvio, arresto e procedura di recupero.

Una nuova demo è pronta quando può essere compilata da un clone pulito,
eseguita sul laptop con l'override e avviata sul NUC con il solo Compose base.

## Simulazione e robot reale

In simulazione `/odom` deriva dalla posa ground-truth di Gazebo. Sul robot
reale il nodo `ground_truth_odom` non deve essere usato: l'odometria deve
provenire dalla cinematica delle ruote, eventualmente fusa con l'IMU.

Riutilizzabili sul robot reale:

- configurazione SLAM Toolbox;
- configurazione Nav2 dopo la taratura di velocità, footprint e costmap;
- missione e waypoint YAML;
- gestione audio.

Esclusivi della simulazione:

- `worlds/demo0_world.sdf`;
- bridge Gazebo;
- odometria ground-truth;
- mappa pre-generata del mondo Demo 0.

Un futuro comando linguistico dovrebbe selezionare una missione valida, non
generare direttamente velocità delle ruote. Nav2 e l'orchestratore possono
così continuare a eseguire comportamenti deterministici e verificabili.
