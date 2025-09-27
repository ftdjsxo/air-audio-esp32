# sketch_sep26a

Firmware per un nodo sensore interattivo "Air Volume" basato su ESP32-WROOM-32D. Il programma campiona un potenziometro, pilota un LED RGB tramite il periferico PWM LEDC e pubblica le letture via Wi-Fi attraverso un captive portal e un feed WebSocket. La logica è ottimizzata per restare reattiva senza mettere in difficoltà il driver radio grazie a un controllo attento della frequenza della CPU e dell'I/O.

## Componenti necessari
- Modulo ESP32-WROOM-32D (o DevKitC che lo monta), con supporto dual core e LEDC
- Potenziometro da 10 kΩ collegato all'ingresso ADC1 su `VP` / `GPIO36` (VP)
- LED RGB (anodo o catodo comune) con resistori di limitazione, collegato a `D18` (verde), `D17` (blu), `D16` (rosso)
- Pulsante momentaneo su `D19` con pull-up interno abilitato
- Alimentazione USB 5 V (oppure 5 V → 3,3 V regolati per uso stand-alone)

## Mappa rapida dei pin (come nel codice)
| Funzione            | Alias Arduino | GPIO ESP32 |
|---------------------|---------------|------------|
| Potenziometro wiper | VP            | IO36 (ADC1_CH0 / VP) |
| LED Verde           | D18           | IO18 (HSPI CLK)      |
| LED Blu             | D17           | IO17 (HSPI MISO)     |
| LED Rosso           | D16           | IO16 (HSPI CS)       |
| Pulsante            | D19           | IO19 (HSPI MOSI)     |

> Nota: il core Arduino-ESP32 espone i define `Dxx` usati nello sketch (`greenPin = D18`, ecc.). Sulle schede DevKitC la serigrafia riporta anche il corrispondente numero GPIO.

## Collegamenti
- **Alimentazione:** collega il pin `5V` (o `VIN`) del DevKitC alla sorgente e unisci tutte le masse a `GND`. Se alimenti a 3,3 V, regola di conseguenza i resistori del LED.
- **Potenziometro:** collega il cursore centrale a `VP`/`GPIO36 (VP)`. I terminali esterni vanno a `3V3` e `GND`, creando un partitore letto dall'ADC1.
- **LED RGB:**
  - LED a **anodo comune**: collega l'anodo a `3V3`. I catodi verde/blu/rosso passano attraverso resistori (~220 Ω) verso `D18`, `D17`, `D16`.
  - LED a **catodo comune**: collega il catodo a `GND` e porta ciascun anodo con resistore ai pin `D18/D17/D16`. In tal caso inverti la logica PWM nel codice se vuoi pilotaggio attivo-alto.
- **Pulsante:** collega un terminale a `D19` e l'altro a `GND`. Il firmware attiva la pull-up interna, quindi la linea rimane HIGH a riposo e va LOW alla pressione.
- Mantieni i cablaggi analogici (potenziometro) lontani dall'antenna Wi-Fi per ridurre il rumore su `VP`.

## Compilazione e caricamento
1. Apri la cartella del progetto nell'Arduino IDE (o CLI) e verifica di avere installato il core `ESP32`.
2. Seleziona un profilo scheda ESP32 compatibile con `D16/D17/D18/D19` e l'ADC1 (es. `ESP32 Dev Module`).
3. Collega la scheda via USB, scegli la porta seriale corretta e carica `sketch_sep26a.ino`.
4. Apri il monitor seriale a 115200 baud per osservare i log diagnostici (`SLog`).

> Suggerimento: fuori dall'IDE puoi esportare il binario (`.bin`) ed eseguire il flash con `esptool.py`.

## Come funziona il firmware
### Sequenza di boot (`setup`)
- Inizializza il logging seriale, la memoria `Preferences` e i pin (timer LEDC, attenuazione ADC, ingresso pulsante).
- Carica eventuali credenziali Wi-Fi salvate; se presenti tenta la connessione in modalità `WIFI_STA`, altrimenti attende input dell'utente.
- Avvia due task FreeRTOS sul core 1: `samplingTask` (alta priorità) e `ledTask` (media).
- Avvia il server TCP WebSocket sulla porta 81, disabilita il Wi-Fi sleep e memorizza timestamp di riferimento.

### Pipeline di campionamento (`samplingTask`)
- Campiona il potenziometro con intervallo variabile a seconda che il dispositivo sia in idle o in modalità interattiva.
- Mantiene uno storico circolare dei delta per calcolare un indice di attività (`0.0‒1.0`).
- Pubblica l'ultima lettura (raw, tensione, percentuale, attività) in una coda a singolo elemento per gli altri componenti.
- Rileva variazioni rapide (due delta consecutivi oltre `IMMEDIATE_TRIGGER_RAW`) e richiede un boost della CPU per restare reattivo.

### Rendering LED (`ledTask`)
- Quando non mostra il potenziometro in tempo reale, il LED indica lo stato Wi-Fi: rosso pulsante in disconnessione, blu "breathing" in ricerca, verde fisso quando collegato.
- In modalità "show pot" il LED passa gradualmente da verde→blu→rosso in base alla percentuale del potenziometro, aggiornando il duty PWM LEDC.

### Loop principale (`loop`)
- Gestisce il debounce del pulsante su `D19`. Alla pressione cancella le credenziali Wi-Fi salvate, forza la disconnessione STA e avvia subito il captive portal.
- Tiene traccia dello stato Wi-Fi: quando connesso chiude il captive portal e mostra il valore del potenziometro; in caso di disconnessione avvia un timer di grazia (`STA_TO_CAPTIVE_DELAY_MS`) prima di aprire il captive AP.
- Serve il captive portal (`WebServer` + `DNSServer`) quando attivo, permettendo di inserire nuove credenziali SSID/password.
- Gestisce i client WebSocket:
  - Accetta fino a quattro client, elaborando l'handshake HTTP a blocchi (`MAX_HS_READ_PER_ITER`).
  - Dopo l'upgrade, incapsula payload JSON e li mette in coda per ogni client, rispettando `availableForWrite()` per restare non bloccante.
- Implementa una cadenza di broadcast adattiva: variazioni significative o heartbeat periodici generano aggiornamenti; troppo traffico attiva un backoff esponenziale.
- Coordina il cambio di frequenza CPU, passando a `NORMAL_CPU_MHZ` durante l'interazione o il lavoro Wi-Fi e tornando a `MIN_IDLE_CPU_MHZ` dopo inattività.

### Flusso Wi-Fi e captive portal
- Le credenziali risiedono nel namespace `airvol` di `Preferences`. Il salvataggio via `/save` forza un nuovo tentativo di connessione STA.
- L'SSID del captive portal `Air Volume` si attiva su pressione del pulsante o allo scadere della grazia dopo una disconnessione inattesa.
- Durante un tentativo di connessione il loop attende fino a `CONNECT_TRY_TIMEOUT_MS` prima di tornare alla modalità captive. Al successo disattiva l'AP, accende il verde fisso e riprende lo streaming del potenziometro.

### Modalità d'interazione
- **Idle:** campionamento e broadcast lenti, CPU a 80 MHz, pensata per il funzionamento stabile.
- **Interactive:** entra quando l'attività supera `ACTIVITY_MIN_FOR_INTERACT` o compaiono delta rapidi. Aumenta frequenza di campionamento/broadcast e forza il boost CPU.
- **Backoff:** scatta se i frame WebSocket superano `BROADCAST_MAX_PER_WINDOW`; il loop dilata l'intervallo e torna normale dopo `BACKOFF_RECOVERY_MS`.

### Diagnostica
- `SLog` invia circa ogni 800 ms una riga compatta con modalità attuale, attività, intervallo di broadcast, frequenza CPU, stato LED e connessione: utile per il debug via seriale.

## Struttura della cartella
```
.
├── README.md
├── sketch_sep26a.ino      # firmware principale
└── captive_pages.h        # frammenti HTML per il captive portal
```

## Idee future
- Salvare costanti di calibrazione del potenziometro per migliorare la scala percentuale.
- Aggiungere aggiornamenti OTA o provisioning BLE per evitare il captive portal.
- Integrare un buffer di metriche per analisi offline quando il Wi-Fi non è disponibile.
