# K1 LiDAR: cosa fa, come funziona e cosa fa meglio del sistema di serie

Spiegazione in parole semplici. Per l'uso passo passo vedi la [guida](GUIDA_IT.md).

> Le osservazioni sul sistema Creality vengono da **prove nostre sulla stessa stampante**
> (firmware 1.3.5.22), non da documentazione ufficiale. Possono cambiare con altri firmware.

---

## 1. Il sistema in breve

La K1 ha un piccolo **laser (LiDAR)** montato sulla testa. Misura l'altezza di quello che c'è sul piatto, punto per punto, come uno scanner. Creality lo usa per due cose:

1. **Pressure Advance (PA)**: trovare il valore che rende gli angoli nitidi, senza rigonfiamenti né buchi.
2. **Primo strato**: controllare che sia stampato bene e, se non lo è, fermare la stampa.

Noi usiamo lo stesso laser, ma con **software nostro** che gira direttamente sulla stampante (dentro Klipper/Moonraker). Si comanda da Fluidd con dei pulsanti `LIDAR_...`.

### Le parti

| Parte | Cosa fa |
|---|---|
| **Driver del laser** (`laser_driver.py`) | Parla direttamente col laser via seriale, senza passare dal programma di servizio di Creality. |
| **Sessione** (`k1lidar/session.py`) | Muove la testa, accende il laser, fa la scansione e registra dove si trova la testa a ogni misura. |
| **Pattern PA** (`pa_pattern.py`) | Genera il gcode del piccolo disegno di prova. |
| **Analizzatore PA** (`algo/pa_phys.py`) | Legge la scansione e calcola il PA. |
| **Analizzatore primo strato** (`fl_analyze.py`) | Confronta lo strato stampato con quello previsto dal gcode. |
| **Archivio tarature** (`store.py`) | Salva il PA per ogni filamento e temperatura. |
| **Job di avvio stampa** (`job.py`) + **hook** | Prima di una stampa legge il gcode, applica il PA giusto, controlla il primo strato. |
| **Componente Moonraker** | Collega i pulsanti di Fluidd al resto e mostra i risultati. |

![Come sono collegate le parti](img/01-architettura.png)


---

## 2. Come funziona la taratura del PA

**Cos'è il PA.** Quando la testa rallenta in un angolo, nell'ugello c'è ancora pressione e il filamento continua a uscire: l'angolo si gonfia. Il PA dice a Klipper di "anticipare" l'estrusione. Troppo poco: angoli tondi e gonfi. Troppo: angoli con buchi.

**Cosa succede quando premi `LIDAR_PA_CALIBRATE`** (circa 15 minuti):

1. **Livellamento** come in fabbrica e scansione del piatto nudo (la base di confronto).
2. **Stampa di un pattern**: 8 righe a zig-zag sul bordo sinistro. Ogni riga è stampata con un PA diverso, da un valore basso a uno alto.
3. **Raffreddamento** dell'ugello (come fa Creality: così le colature non sporcano la misura).
4. **Scansione del pattern** con il laser, in un'unica passata continua.
5. **Analisi**: si cerca la riga in cui gli angoli sono i più "giusti".
6. Il valore si **salva per quel filamento** e viene applicato.

![Flusso della taratura PA](img/02-flusso-pa.png)


Il filamento si riconosce dal **nome del profilo dello slicer** (scritto nel gcode) più la temperatura dell'ugello. Due filamenti diversi vogliono due profili con nomi diversi.

---

## 3. Pressure Advance: cosa non andava e come l'abbiamo corretto

### Problema 1 — La posizione delle foto era sbagliata
**Cosa succede.** Per scansionare, la testa fa una passata e il laser scatta tante "foto" del profilo. Il metodo di serie dà per scontato che le foto siano perfettamente regolari e ricava *dove* è stata scattata ognuna dal suo **numero** ("la foto n.50 è a 50 passi dall'inizio").

Nella pratica non lo sono:
- Il sensore scatta circa **11 foto al secondo**, e il loro numero cambia da un run all'altro (nelle nostre prove 162, 165, 167).
- Klipper, quando gli dai un movimento lungo, **non risponde subito**: aspetta che in coda resti poco movimento. Se i tempi di scatto partono dalla risposta, partono in ritardo: arrivano foto scattate **a testa ferma** a fine corsa. Il log diceva "ritardo 0 ms, 170/170 foto" ma erano sbagliate.

Risultato: la stessa riga di pattern dava valori diversi da un run all'altro, e alcune tarature erano invalide senza che nulla lo segnalasse.

**Come l'abbiamo corretto.** Un solo movimento continuo. Ogni foto ha un'ora precisa; poi si ricava **dove era davvero la testa in quell'istante** leggendo la traiettoria reale pianificata da Klipper. La velocità si abbassa da sola (default F400) per avere almeno una foto ogni mezzo passo. Per ogni posizione che serve si prende la foto più vicina.
Misurato a vuoto: 340 foto, errore di posizione massimo 0,3 mm, nessuna foto mancante.

![Posizione delle foto: prima e dopo](img/03-posizione-foto.png)

### Problema 2 — Il risultato dipendeva dalla posizione, non dal PA
**Cosa succede.** Stesso filamento (PLA), due prove pulite a 20 minuti di distanza: l'analisi di serie ha dato **0,054** e **0,0375**. Il valore seguiva *in quale punto del pattern* era la riga migliore, non il PA. Con il PLA il range di fabbrica finiva poi sulla prima riga, che si stampa peggio perché la testa riparte dopo uno spostamento.

**Come l'abbiamo corretto.** Un analizzatore nuovo, basato sulla fisica dell'angolo:
- si misura il **volume di materiale** intorno a ogni angolo (±1,5 mm), usando **tutte** le foto e non pochi punti;
- si confronta il **riempimento dello spigolo** con quello del tratto dritto: più PA c'è, meno materiale resta nell'angolo, quasi in linea retta;
- dalla retta si ricava il PA, togliendo l'effetto "posizione nel pattern";
- si ripete su sottoinsiemi dei dati per dare un **intervallo di confidenza**, per esempio `0,0274 (0,0230–0,0303)`.

![Come si trova il PA dagli angoli](img/04-riempimento-angolo.png)

Ripetibilità misurata, stesse condizioni: **0,0274** e **0,0270**. L'analisi di serie sugli stessi dati: 0,054 / 0,0375 / 0,036. Il sistema di serie non viene più usato.

![Ripetibilità: Creality contro il nostro](img/05-ripetibilita.png)

### Problema 3 — Nessuna protezione dai dati incompleti
**Cosa succede.** Se mancano dati in qualche riga, viene dato comunque un numero.
**Come l'abbiamo corretto.** Risultato scartato con un messaggio chiaro ("righe 3-8 mancanti"). Se il valore cade fuori dall'intervallo provato non viene salvato e ti dice con quale `START=` riprovare. Se la confidenza è bassa ti avvisa.

### Problema 4 — Un solo PA per tutto, range unico
**Come l'abbiamo corretto.** Il PA si salva **per profilo filamento e temperatura**. Il range cambia per materiale: PLA/ABS/ASA 0,006–0,0525; PETG 0,016–0,078; TPU 0,05–0,205. Il pattern alza la testa a 5 mm negli spostamenti (niente urti sulle righe), e le scansioni restano a Z3, la quota giusta per il laser. `FINE=1` aggiunge un secondo pattern stretto intorno al primo risultato.

### Altre cause di errore che abbiamo scoperto
- **Il livellamento conta.** Un livellamento diverso da quello di fabbrica sposta lo zero in Z e cambia il modo in cui l'analisi legge gli angoli (0,08 contro 0,032 nelle nostre prove). Per questo si usa sempre il livellamento di fabbrica.
- **Le colature dell'ugello falsano la misura.** Prima della scansione l'ugello si raffredda e le ventole vanno al massimo (0,04 senza, 0,034 con).

---

## 4. Primo strato: cosa non andava e come l'abbiamo corretto

Il flusso è lo stesso del display: scansione del piatto nudo, stampa del solo primo strato, pausa, raffreddamento, seconda scansione sulla stessa traiettoria, confronto con il gcode. Poi il verdetto: **buono** → si riprende; **difettoso** → pausa o annullamento, indicando **dove** (in mm).

![Flusso del controllo del primo strato](img/06-primo-strato.png)

### Problema 1 — Il verdetto di serie è un sì/no, e si inganna
Nelle nostre prove, **togliendo materiale** da una zona del pezzo, il detector di serie è passato da "ok" a "problema": ha reagito al contrario di come dovrebbe. Non dice dove sta il difetto.

**Correzione.** Un analizzatore nostro (C2) costruisce una **mappa delle altezze** allineata al gcode e cerca zone dove il materiale **manca** o è **sollevato**. Per ogni difetto dà posizione e area in mm². Sui nostri pezzi ha trovato 13 mm² di mancante e 9 mm² di sollevamento dove il sistema di serie non segnalava nulla. Il verdetto di serie viene comunque calcolato e stampato accanto, così ogni divergenza si vede.

### Problema 2 — Piccoli spostamenti creano difetti fantasma
- Lo **zero in Z cambia con la temperatura** anche di 0,1 mm da un run all'altro. C2 toglie lo spostamento globale del piano e continua a vedere difetti da pochi mm².
- C2 **si allinea da solo** al gcode (cerca lo scostamento che fa combaciare meglio).
- Il sensore ha una **distorsione fissa** (circa 3 mm lungo la lama laser) che causava falsi "materiale mancante" dove le fasce si sovrappongono. C2 la misura durante la scansione.
- I **riflessi** su superfici lucide vengono filtrati.
- Per oggetti larghi si usano **più fasce** (il laser vede una striscia di 26–28 mm).

### Problema 3 — Dati insufficienti
Se i dati non bastano, C2 risponde **INCERTO** e **non riprende mai da solo**.

### Problema 4 — La stampa si fermava a ogni strato
Dopo la pausa del primo strato il firmware teneva il flag "primo strato" e fermava la stampa a ogni cambio strato, senza finire mai. Appena la stampa va in pausa, azzeriamo il flag con lo stesso meccanismo del display.

---

## 5. Cosa si può fare, in pratica

Da Fluidd → Macro:

| Pulsante | Cosa fa |
|---|---|
| `LIDAR_PA_CALIBRATE GCODE="file.gcode"` | Trova il PA di quel filamento, senza stampare un oggetto. Prende nome profilo e temperature dal gcode. |
| `LIDAR_PA_LIST` / `SET` / `FORGET` | Vedi, imposta a mano o cancella i PA salvati. |
| `LIDAR_FLOW`, `LIDAR_FIRSTLAYER` | Attivano i controlli prima di una stampa (serve l'hook). |
| `LIDAR_HELP` | Guida rapida in console. |

**Ordine con cui si sceglie il PA:** (1) quello scritto nel gcode, se lo slicer ha il PA attivo; (2) quello misurato per quel profilo e temperatura; (3) se non c'è e `LIDAR_FLOW` è attivo, lo misura adesso; (4) altrimenti quello di `printer.cfg`.

Consiglio: nello slicer **disattiva "Enable pressure advance"**, altrimenti sovrascrive il valore misurato.

---

## 6. Stato e limiti (da dire onestamente)

- Il **PA** è validato sulla stampante con più run ripetuti. Il **primo strato C2** è validato su 3 run reali; il flusso completo "avvio stampa → hook → primo strato → stampa finita" **non è ancora provato da capo a fondo**.
- Il sistema vale solo per **K1 con il modulo LiDAR**, provato sul firmware 1.3.5.22. Dopo un aggiornamento del firmware va rilanciato `install.sh` (l'update può riscrivere `printer.cfg`).
- Il laser dà una misura solo se la testa è a **Z3 e dopo l'homing**; senza, i frame sono vuoti (normale).
- Il **repo pubblico** su GitHub non è ancora stato creato.
