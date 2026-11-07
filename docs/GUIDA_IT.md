# K1 LiDAR — guida in italiano

Taratura del **Pressure Advance** e **controllo del primo strato** con il modulo LiDAR
(laser) della **Creality K1**, da Klipper, con Fluidd o Mainsail.

> English: [MANUAL.md](MANUAL.md)

## Indice
1. [Cosa fa](#cosa-fa)
2. [Requisiti](#requisiti)
3. [Installazione](#installazione)
4. [Profili filamento: come il PA si lega allo slicer](#profili-filamento-come-il-pa-si-lega-allo-slicer)
5. [Quale PA usa una stampa](#quale-pa-usa-una-stampa)
6. [Trovare il PA di un filamento (senza stampare)](#trovare-il-pa-di-un-filamento-senza-stampare)
7. [Taratura prima della stampa](#taratura-prima-della-stampa)
8. [Tutti i pulsanti](#tutti-i-pulsanti)
9. [Come viene misurato il PA](#come-viene-misurato-il-pa)
10. [Come leggere il risultato](#come-leggere-il-risultato)
11. [Sicurezza](#sicurezza) · [Come funziona](#come-funziona) · [Stato e limiti](#stato-e-limiti) · [Problemi comuni](#problemi-comuni)

## Cosa fa
- **Trova il PA di un filamento** con un pulsante, senza stampare un oggetto: la
  stampante stampa un piccolo pattern di prova sul bordo sinistro del piatto, lo
  scansiona col laser e ti dà il PA con il suo margine d'errore. Il valore resta
  **salvato per quel profilo filamento** e viene riusato da solo.
- **Prima di una stampa** (opzionale): profilo filamento e temperature si leggono dal
  gcode. Se il profilo è già tarato si applica subito il suo PA; se è nuovo si tara
  prima. Se vuoi, il **primo strato viene scansionato** e la stampa va in pausa (o si
  annulla) se è difettoso, con la posizione del difetto in mm.

## Requisiti
- **K1 con modulo LiDAR** (model CR-K1): l'installer lo verifica. K1 Max e K1C non sono
  supportate (non provate).
- **Root** e **Moonraker + Fluidd o Mainsail** (per esempio con il
  [Creality Helper-Script](https://github.com/Guilouz/Creality-Helper-Script)).
- Provato sul firmware **1.3.5.22**; con altre versioni l'installer avvisa.
- Non serve scaricare nulla sulla stampante: Python 3.8 e numpy sono nel firmware.
- Uno slicer che scrive le impostazioni in fondo al gcode: OrcaSlicer, Creality Print,
  PrusaSlicer, SuperSlicer, Bambu Studio.

## Installazione
Sulla stampante, via SSH come root:
```sh
git clone https://github.com/<utente>/k1-lidar /usr/data/k1-lidar
sh /usr/data/k1-lidar/install.sh --with-hook --lang it
```
L'installer controlla i requisiti, fa il backup di `printer.cfg`, collega macro e
componente Moonraker, aggiunge la voce nell'*Update manager* e riavvia Klipper e
Moonraker. Si può rieseguire senza problemi.

| Opzione | |
|---|---|
| `--with-hook` | attiva le funzioni "prima della stampa" (intercetta `SDCARD_PRINT_FILE`; con tutto spento non cambia nulla). Senza, hai solo i pulsanti. |
| `--no-hook` | toglie l'hook (stampe di nuovo standard) |
| `--lang it\|en` | lingua dei messaggi |
| `--check` | controlla solo i requisiti, non tocca nulla |

- **Aggiornamenti**: Fluidd/Mainsail → *Update manager* → **k1-lidar**.
- **Dopo un aggiornamento firmware Creality** rilancia `install.sh` (l'aggiornamento
  riscrive `printer.cfg` e può cancellare `moonraker.conf`).
- **Disinstallazione**: `sh /usr/data/k1-lidar/uninstall.sh` (`--purge` cancella anche
  le tarature salvate).

Dopo l'installazione apri Fluidd → **Macro**: ogni pulsante `LIDAR_...` mostra una
descrizione di cosa fa. **`LIDAR_HELP`** stampa una guida rapida in console.

## Profili filamento: come il PA si lega allo slicer
Il PA viene salvato **per profilo filamento dello slicer e per temperatura dell'ugello**:

```
"Creality HF Generic Speed PLA - Copia" a 220 °C  ->  PA 0.0274
"Creality Generic PETG+HS ESUN"         a 245 °C  ->  PA ...
```

- Il **nome del profilo** è quello che scegli nella casella *Filamento* dello slicer.
  Lo slicer lo scrive nel gcode (`filament_settings_id`): così all'avvio della stampa
  la stampante sa quale filamento è caricato e cerca il suo PA.
- La **temperatura** è quella degli strati dopo il primo. Lo stesso profilo usato a
  un'altra temperatura è una voce diversa (il PA cambia con la temperatura).
- **Due filamenti diversi vogliono due profili diversi**, anche se stampano alla stessa
  temperatura: nello slicer duplica il profilo e dagli un nome chiaro, per esempio
  "PLA eSun arancione" e "PLA Sunlu nero". È il nome che li distingue: se li stampi
  entrambi con lo stesso profilo, condividono lo stesso PA.
- **Disattiva "Enable pressure advance"** nel profilo filamento dello slicer, altrimenti
  lo slicer scrive un suo PA nel gcode e sovrascrive quello misurato.

Dove si salva: `/usr/data/lidar_calib.json` sulla stampante (fuori dalla cartella del
toolkit, così gli aggiornamenti non lo toccano). Si gestisce con i pulsanti:

| Pulsante | |
|---|---|
| `LIDAR_PA_LIST` | elenca i profili con un PA salvato: nome, temperatura, PA, intervallo, data |
| `LIDAR_PA_SET PA=0.027 GCODE="file.gcode"` | imposta a mano il PA del profilo usato in quel gcode (oppure `FILAMENT="nome esatto del profilo" NOZZLE=220`) |
| `LIDAR_PA_FORGET FILAMENT="nome profilo" [NOZZLE=220]` | lo cancella (senza `NOZZLE`: tutte le temperature) |

## Quale PA usa una stampa
In quest'ordine — vince il primo che vale:

1. **Il PA del gcode**, se nel profilo dello slicer è attivo *Enable pressure advance*.
   In console compare un avviso.
2. **Il PA misurato col LiDAR** per quel profilo e quella temperatura, se c'è. Viene
   applicato appena prima della stampa (`LIDAR_PA_APPLY ON=1`, il default; serve l'hook).
3. Se `LIDAR_FLOW` è attivo e il profilo non ha ancora un PA: **viene misurato adesso**,
   prima della stampa, poi salvato e applicato.
4. Altrimenti **il valore di `printer.cfg`** (`pressure_advance` in `[extruder]`).

Nota: la *modalità silenziosa* del display, se la attivi durante la stampa, imposta PA
0.05 finché non la spegni.

## Trovare il PA di un filamento (senza stampare)
Carica il filamento, libera il bordo sinistro del piatto (X 0–30 mm), poi in Fluidd →
Macro:

**Consigliato — da un gcode** (carica un qualsiasi file fatto con quel profilo filamento):
```
LIDAR_PA_CALIBRATE GCODE="mio_pezzo_PLA.gcode"
```
Nome del profilo, materiale e temperature vengono presi dal gcode: il risultato si
salva esattamente sotto il nome che cercheranno le stampe con quel profilo. Il file
viene solo letto, non stampato.

**A mano**:
```
LIDAR_PA_CALIBRATE FILAMENT="Creality HF Generic Speed PLA - Copia" MATERIAL=PLA NOZZLE=220 BED=60
```
`FILAMENT` deve essere **esattamente** il nome del profilo nello slicer, altrimenti le
stampe non lo trovano. Senza `FILAMENT` né `GCODE` il PA si salva solo col nome del
materiale (es. `PLA_T220`) e ricevi un avviso.

- Dura circa 15 minuti: livellamento di fabbrica, scansione del piatto nudo, pattern di
  prova, raffreddamento, scansione del pattern, analisi.
- Il PA viene applicato subito e salvato. In console compare per esempio
  `Risultato PA LiDAR: 0.0274 (intervallo 80% 0.0230-0.0303), confidenza alta`.
- Opzionale: `FINE=1` aggiunge un secondo pattern 30 mm più a destra, a passi di 0.001
  attorno al primo risultato (tieni libera anche quella zona). `START=` / `STEP=`
  cambiano l'intervallo dei 32 valori di prova (vedi
  [Come viene misurato il PA](#come-viene-misurato-il-pa)).

## Taratura prima della stampa
Serve l'hook (`install.sh --with-hook`). In Fluidd → Macro premi **una volta**:

| Pulsante | Cosa succede a ogni avvio stampa |
|---|---|
| `LIDAR_FLOW MODE=1` | **consigliato**: i profili mai misurati vengono tarati prima della stampa (~15 min, solo la prima volta); per gli altri si applica subito il PA salvato |
| `LIDAR_FLOW MODE=2` | tara prima di **ogni** stampa |
| `LIDAR_FLOW MODE=0` | spento (il PA salvato viene comunque applicato, vedi `LIDAR_PA_APPLY`) |
| `LIDAR_FIRSTLAYER ON=1 ON_BAD=pause` | controlla anche il primo strato (`ON_BAD=cancel` annulla una stampa difettosa) |
| `LIDAR_PA_APPLY ON=0` | non applicare i PA salvati (usa `printer.cfg` / il gcode) |

La scelta resta (anche dopo un riavvio); `LIDAR_STATUS` mostra cosa è attivo. Poi avvia
le stampe come sempre — da Fluidd, Mainsail o dal display. Cosa succede:

1. Nome del profilo, materiale e temperature si leggono dal gcode (altrimenti da
   `START_PRINT EXTRUDER_TEMP= BED_TEMP=`; `LIDAR_SET_MATERIAL` imposta un ripiego).
2. PA salvato per quel profilo e quella temperatura → applicato, nessun pattern.
   Altrimenti, con `LIDAR_FLOW` attivo, il piatto viene livellato, il pattern di prova
   viene stampato sul bordo sinistro, scansionato, e il PA viene salvato e applicato.
3. Controllo primo strato (se attivo): lo strato viene stampato, la stampa va in pausa,
   lo strato viene scansionato: **GOOD** → riprende; **BAD** → pausa (o annulla);
   **UNCERTAIN** → sempre pausa. Il piatto si livella una volta sola.
4. La stampa prosegue normalmente.

Tieni libero il bordo sinistro del piatto quando può partire una taratura.

## Tutti i pulsanti
| Pulsante | |
|---|---|
| `LIDAR_HELP` | guida rapida in console |
| `LIDAR_STATUS` | impostazioni attuali (e se l'hook è installato) |
| `LIDAR_PA_CALIBRATE` | trova il PA adesso: `GCODE=` oppure `FILAMENT=`, `MATERIAL=`, `NOZZLE=`, `BED=`, `FINE=1`, `START=`, `STEP=` |
| `LIDAR_PA_SET` | imposta un PA a mano: `PA=` e `GCODE=` oppure `FILAMENT=` + `NOZZLE=` |
| `LIDAR_PA_LIST` / `LIDAR_PA_FORGET` | elenca / cancella i profili salvati |
| `LIDAR_FLOW MODE=0/1/2` (`LIDAR_FLOW_ON` / `_OFF`) | taratura prima della stampa |
| `LIDAR_FIRSTLAYER ON=0/1 ON_BAD=pause/cancel` (`_ON` / `_OFF`) | controllo primo strato |
| `LIDAR_PA_APPLY ON=0/1` | applica il PA salvato all'avvio stampa |
| `LIDAR_SET_MATERIAL MATERIAL= HOTEND= BED=` | ripiego se il gcode non lo dice |
| `LIDAR_CANCEL` | ferma il job in corso; laser e riscaldatori spenti |
| `LIDAR_LANGUAGE LANG=it/en` | lingua dei messaggi |

API HTTP per altri strumenti: `GET /machine/k1lidar/status`, `GET /machine/k1lidar/filaments`,
`POST /machine/k1lidar/settings`, `POST /machine/k1lidar/pa`, `POST /machine/k1lidar/set`,
`POST /machine/k1lidar/print`, `POST /machine/k1lidar/cancel`.

## Come viene misurato il PA
Il pattern di prova è fatto di 4 colonne di zig-zag; ogni **spigolo** dello zig-zag è
stampato con un PA diverso (32 valori). In uno spigolo stretto la testa frena e
riaccelera: con PA troppo basso lo spigolo fa un grumo, con PA troppo alto l'estrusore
ritrae troppo e lo spigolo diventa **sottile o vuoto**. Il laser misura, per ogni
spigolo, il volume del cordone nello spigolo rispetto al tratto dritto della stessa
linea (il *riempimento*). Il riempimento cala con regolarità al crescere del PA; il PA
scelto è quello a cui lo spigolo è **pieno quanto il tratto dritto**. I 32 spigoli si
analizzano tutti insieme, si toglie un piccolo effetto noto della posizione dello
spigolo nella riga, e un bootstrap dà l'intervallo all'80%.

Intervallo predefinito dei 32 valori — scelto perché il valore atteso cada a metà
pattern, dato che le prime righe si stampano un po' peggio:

| Materiale | Intervallo | Passo |
|---|---|---|
| PLA, ABS, ASA | 0.006 – 0.0525 | 0.0015 |
| PETG | 0.016 – 0.078 | 0.002 |
| TPU (sperimentale) | 0.050 – 0.205 | 0.005 |

## Come leggere il risultato
- **confidenza alta**: usalo.
- **confidenza BASSA**: il valore viene salvato e applicato, ma la console elenca il
  perché (scansione rumorosa, andamento debole, intervallo largo). Ripeti con il piatto
  pulito.
- **"PA fuori dal range del pattern"**: la stima cade fuori dai valori stampati, quindi
  non è una misura — non viene salvato nulla. Ripeti con lo `START=` suggerito.
- **Errori di scansione** (righe mancanti, scansione vuota) fermano la taratura: non
  viene salvato nulla.

## Sicurezza
- Tieni libero il bordo sinistro del piatto (X 0–30 mm, 60 mm con `FINE=1`) quando può
  partire una taratura.
- Tutti gli spostamenti del pattern di prova avvengono a Z 5 mm.
- `LIDAR_CANCEL` ferma un job e spegne laser e riscaldatori; un job che fallisce spegne
  comunque i riscaldatori.
- L'hook è opzionale: se qualcosa non va, `install.sh --no-hook` (o togli
  `[include k1lidar_hook.cfg]` da `printer.cfg`) e le stampe tornano standard.
- Le scansioni grezze di ogni run restano in `/usr/data/lidar_archive/` (utili per
  segnalare problemi).

## Come funziona
- Il modulo laser è comandato direttamente sulla sua porta seriale; il middleware
  Creality è solo un ripiego.
- La scansione è una passata lineare continua, come il firmware di serie. La posizione
  di ogni profilo laser viene dalla coda di moto di Klipper
  (`motion_report/dump_trapq`) sull'orologio di Klipper: ogni profilo è collocato entro
  ~0.3 mm. Il sensore dà ~11 profili al secondo; la velocità si adatta per averne uno
  ogni ~0.6 mm.
- L'analisi del PA (`algo/pa_phys.py`) e quella del primo strato (`fl_analyze.py`: mappa
  delle altezze allineata al gcode, zone mancanti/sollevate) fanno parte di questo
  progetto. L'analisi PA di Creality **non** viene usata: sullo stesso filamento dava
  0.054 o 0.0375 a seconda di dove cadeva il valore nel pattern.

## Stato e limiti
- Validato su una K1 con PLA (220/60 °C): due tarature indipendenti della stessa bobina
  hanno dato **0.0274** (intervallo 80% 0.0230–0.0303) e **0.0270** (0.0233–0.0310).
  L'analisi Creality sulle stesse scansioni: 0.054, 0.0375, 0.036.
- Gli intervalli per PETG/ABS/ASA sono valori di partenza ragionati, il TPU è
  sperimentale: segnala i risultati.
- Solo la K1 (CR-K1) è supportata dall'installer.

## Problemi comuni
| Sintomo | Cosa fare |
|---|---|
| una stampa non usa il PA misurato | `LIDAR_PA_LIST`: il nome del profilo è uguale a quello dello slicer, e la temperatura è la stessa? *Enable pressure advance* è spento nello slicer? `LIDAR_STATUS` dice che l'hook è installato? |
| `LiDAR: c'e' gia' un'operazione in corso` | aspetta, oppure `LIDAR_CANCEL` |
| pulsanti assenti in Fluidd | rilancia `install.sh`; controlla `[include k1lidar.cfg]` in `printer.cfg` |
| Fluidd smette di funzionare dopo un aggiornamento firmware | l'aggiornamento ha cancellato `moonraker.conf`: rilancia `install.sh` |
| laser non trovato | `sh /usr/bin/laser_power.sh on` e riprova; controlla il cavo del modulo |
| risultato con confidenza BASSA | pulisci il piatto, verifica che le prime righe siano stampate bene, ripeti |

Non affiliato a Creality. Uso a tuo rischio: muove la stampante, la scalda e stampa.
Licenza GPL-3.0.
