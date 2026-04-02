# Hardware Validation Roadmap

> **Status:** Active
> **Created:** 2026-03-23
> **Scope:** From single-joint baseline to dual-leg assembly

---

## Principi

1. **Ogni livello ha exit criteria misurabili.** Non si passa al successivo finché non sono soddisfatti.
2. **Multi-DOF prima di multi-joint.** Il salto architetturale più critico è `MAX_DOFS > 1` sullo stesso controller, non il numero di joint.
3. **Webapp per commissioning singolo controller, Jetson per coordinazione multi-controller.**
4. **Una gamba prima di due.** La seconda gamba è una replica, non un esperimento.

---

## L1 — Knee Single (1 DOF, 1 controller)

**Stato:** ✅ Closed as architecture baseline

**Cosa è validato:**
- Impedance control (move stream 50Hz, oscillazione sinusoidale)
- Hold stabile con outer Ki freeze
- RevTrack in dominio attuatore (fix domain mismatch 0x92/0xA1)
- DIAG_HOLD telemetria unificata (bias EMA, torque ratio, motor residual)
- Retension probe con publish CAN
- Webapp move/hold/oscillation path completo
- Watchdog configurabile (hold + stream)
- SLCAN listener stabile (fix `timestamp` crash)

**Cosa NON è chiuso (e non serve per salire di livello):**
- Tension trim attivo (Phase 2) — resta dry-run
- Slack detector definitivo — soglie non ancora validate su dati puliti
- Politica host finale per auto-retension

**Exit criteria:** ✅ Tutti soddisfatti
- [x] Hold a 0°, 30°, 60°, 90° senza watchdog timeout
- [x] Oscillazione stabile per almeno 60s
- [x] RevTrack senza warning persistenti (post-fix)
- [x] DIAG_HOLD visibile in webapp con segnali coerenti
- [x] Nessun crash SLCAN sotto carico normale

---

## L2 — Ankle Single (2 DOF, 1 controller)

**Stato:** ✅ Closed as architecture baseline

**Obiettivo:** Validare firmware e webapp con 2 DOF sullo stesso controller. Primo test di coordinazione intra-joint.

**Piattaforma:** Webapp

**Cosa testa di nuovo rispetto a L1:**
- `dof_state[]` con 2 elementi attivi contemporaneamente
- Hold/move indipendente per DOF (es. DOF0 in hold, DOF1 in movimento)
- DIAG_HOLD per DOF multipli (card UI, chart con DOF selector)
- 4 motori sullo stesso bus CAN motore (vs 2 del ginocchio)
- Pretension/recalc su 2 DOF — sequenza e timing
- Smart impedance buttons con mapping multi-DOF

**Rischi specifici:**
- Contesa bus CAN motore: 4 motori a 50Hz = 200 frame/s solo torque command+response
- Interazione meccanica tra i 2 DOF (un DOF muove il carico dell'altro)
- Sequenza di init/homing: ordine dei DOF conta?

**Exit criteria:** ✅ Tutti soddisfatti sul banco `ANKLE_RIGHT`
- [x] Init + homing di entrambi i DOF senza errori
- [x] Hold stabile su entrambi i DOF contemporaneamente
- [x] Move DOF0 mentre DOF1 è in hold — nessun disturbo rilevante su DOF1
- [x] DIAG_HOLD coerente per entrambi i DOF
- [x] Oscillazione / move di un DOF senza regressioni sull'altro
- [x] Nessun CAN error/overrun sul run nominale validato
- [x] Recalc offset / startup sequence stabile e ripetibile

**Nota pratica:**
- `ANKLE_LEFT` non è disponibile sul banco attuale
- la chiusura L2 è quindi sul profilo `ANKLE_RIGHT`, sufficiente come baseline architetturale single-controller 2-DOF

---

## L3 — Hip Single (3 DOF, 1 controller)

**Stato:** ⏸ Blocked by hybrid-model refactor

**Obiettivo:** Validare il caso più pesante per singolo controller: 3 DOF, 5 motori.

**Piattaforma:** Webapp

**Blocco attuale:**
- la `HIP` reale non è un joint uniforme `3 DOF / 6 motori / 2 motori per DOF`
- il modello corretto è un joint ibrido:
  - 2 DOF tendon-driven antagonistic
  - 1 DOF roll direct-drive single motor
- prima del test singolo HIP va corretta l'astrazione software, come descritto in [HIP_HYBRID_DOF_SPEC.md](software/docs/HIP_HYBRID_DOF_SPEC.md)

**Cosa testa di nuovo rispetto a L2:**
- MAX_DOFS=3 tutti attivi
- 5 motori sullo stesso bus CAN motore (~250 frame/s torque loop)
- Outer PID per 3 DOF concorrenti
- DIAG_HOLD per 3 DOF — massimo carico telemetria per controller
- Timing del control loop: 2 + 2 + 1 motori attivi con un mix tendon/direct-drive = tempo ciclo critico

**Rischi specifici:**
- Control loop non completa entro 2ms con 5 motori (2 tendon pair + 1 direct-drive)
- Bus CAN motore saturo: 5 motori × 50Hz × 2 (cmd+resp) = 500 frame/s
- Pretension/recalc su 2 DOF tendon + startup direct-drive del roll: tempo totale, stabilità
- Heat: 5 motori attivi in hold generano calore — thermal throttling?

**Exit criteria:**
- [ ] Init + homing di tutti e 3 DOF senza errori
- [ ] Hold stabile su 3 DOF contemporaneamente per 120s
- [ ] Control loop rate stabile a 500Hz (misura jitter con log)
- [ ] Nessun CAN error/overrun su bus motore con 5 motori
- [ ] DIAG_HOLD per tutti e 3 DOF: segnali coerenti
- [ ] Temperatura motori stabile dopo 5 min di hold (no thermal runaway)
- [ ] Move un DOF mentre gli altri 2 sono in hold — nessun disturbo

---

## L3.5 — Protocol Freeze Gate

**Stato:** 🔲

**Obiettivo:** Congelare il protocollo CAN host↔controller prima di sviluppare il path Jetson definitivo. Evita di testare L4 con un protocollo in movimento.

**Da congelare:**
- [ ] Command cadence: 50Hz confermato come target production
- [ ] Watchdog policy: valore nominale (100ms Jetson), valore rilassato (500ms webapp), hold watchdog configurabile
- [ ] Telemetria production vs debug: quali stream attivi per default, quali su richiesta
- [ ] Sequence counters: formato e posizione nei frame
- [ ] Error handling: cosa fa il controller quando perde N comandi consecutivi
- [ ] JOINT_STATE: contenuto e rate confermati
- [ ] DIAG_HOLD / RPROBE: rate production confermato

**Deliverable:** Sezione "Protocol v1.0" in CAN_SYSTEM_ARCHITECTURE.md con tutti i valori congelati e versionati.

**Exit criteria:**
- [ ] Documento protocol freeze approvato
- [ ] Nessun campo "TBD" nei frame production
- [ ] Jetson controller software può essere sviluppato contro questa specifica senza ambiguità

---

## L4 — Two-Controller Bench (Jetson)

**Stato:** 🟡 Bench-validated on current protocol, formal freeze still pending

**Obiettivo:** Prima validazione multi-controller coordinato via Jetson. Knee + Ankle su banco (non assemblati meccanicamente).

**Piattaforma:** Jetson (CAN diretto)

**Nota:** Il software Jetson sviluppato qui deve essere il **path finale**, non un prototipo throwaway. Scheduler a 50Hz, comando multi-controller, watchdog discipline, logging strutturato di jitter/latency/error counters.

**Cosa testa di nuovo:**
- Jetson controller software: loop di comando a 50Hz per 2 controller
- Bus CAN host con 2 joint attivi: comandi + feedback + telemetria
- Sincronizzazione temporale dei comandi tra joint
- Watchdog tight (100ms) con Jetson — fattibile?
- Recovery: cosa succede se un controller non risponde?

**Pre-requisiti:**
- **L3.5 protocol freeze completato**
- Jetson controller software funzionante per invio comandi impedance
- Almeno knee + ankle controller flashati e funzionanti singolarmente

**Metriche da raccogliere:**
- Loop rate effettivo del Jetson (target 50Hz)
- Latenza comando→risposta per joint
- Skew tra comandi ai 2 controller (target < 1 periodo = 20ms)
- CAN bus utilization (frame/s, % bandwidth)
- Error counters CAN (overrun, lost arbitration, bus-off)
- Watchdog warnings/timeouts

**Exit criteria:**
- [ ] Jetson invia comandi a 2 controller a 50Hz per 30 min senza timeout
- [ ] Skew tra comandi ai 2 controller < 20ms (1 periodo)
- [ ] Nessun CAN error/overrun sul bus host
- [ ] Recovery da controller restart: il Jetson rileva e ri-inizializza
- [ ] Recovery da Jetson restart: i controller vanno in safe (watchdog)
- [ ] CAN bus utilization < 70% (margine per telemetria)

**Evidenza già raccolta sul banco (2026-03-31):**
- `KNEE_RIGHT + ANKLE_RIGHT` scoperti correttamente sullo stesso bus
- startup nominale multi-controller: valida
- home e nudge manuali su tutti i DOF: validi
- exercise automatico `knee + ankle` per 60s e 300s: valido
- `INVALID CAN READ = 0`, `missed 0xA1 = 0`, `RESYNC = 0`, `PROFILING OVER BUDGET = 0` nei run puliti validati
- post-`E-stop` nella **stessa sessione Jetson**: recovery valido con:
  - `PRETENSION_ALL` selettivo
  - `RECOVERY_SETTLE`
  - resume sulla posa corrente

**Ancora aperto su L4:**
- recovery automatico se il processo Jetson viene riavviato dopo `E-stop`
- formalizzazione del protocol freeze `L3.5` prima di dichiarare L4 "closed" anche a livello documentale

---

## L5 — Three-Controller Stress Test (Jetson)

**Stato:** 🔲

**Obiettivo:** Validare il throughput CAN e il software Jetson con il carico completo di mezza gamba (hip + knee + ankle).

**Piattaforma:** Jetson (CAN diretto), 3 controller su banco

**Profili CAN da definire:**

| Profilo | Comandi | Feedback | Telemetria | Uso stimato |
|---------|---------|----------|------------|-------------|
| **Production** | 50Hz impedance per joint | 50Hz encoder stream | DIAG_HOLD 0.3Hz, RPROBE on-demand | ~baseline |
| **Debug** | come production | come production | + serial log verboso | ~+30% |
| **Stress** | come production | come production | + DIAG_HOLD 2Hz, forced probes | ~+50% |

**Stima bandwidth (ordine di grandezza, non valore di progetto):**

Il costo reale di un frame CAN standard dipende da payload, bit stuffing e
inter-frame spacing. 120 bit/frame è un lower bound; un budget conservativo
è ~130-150 bit/frame effettivi.

A 500 kbps con ~140 bit/frame reali: ~3570 frame/s max pratico.

- Production stima: 3 joint × (50 cmd + 50 resp + 50 encoder + 10 joint_state) = 480 frame/s → ~13%
- Con telemetria debug: +50 frame/s → ~15%

Conclusione: il bus fisico **molto probabilmente** regge. Ma va **dimostrato con
test strumentato** (L5), non assunto. Il vero limite sarà il software host
(jitter di scheduling, contesa tra RX e TX, gestione code).

**Metriche da raccogliere:**
- Tutte quelle di L4, più:
- Distribuzione jitter per-joint (istogramma)
- Worst-case loop time
- Tempo di recovery dopo E-Stop globale
- Durata sessione senza degrado

**Exit criteria — nominale:**
- [ ] 3 controller attivi per 60 min senza errori CAN
- [ ] Loop rate Jetson stabile a 50Hz (jitter < 5ms p99)
- [ ] Nessun frame loss rilevabile (sequence counter check)
- [ ] E-Stop → tutti i controller safe entro 100ms
- [ ] Restart singolo controller → Jetson rileva e ri-inizializza entro 2s
- [ ] Profilo "Production" stabile senza CAN overrun
- [ ] Profilo "Stress" identificata soglia massima sostenibile

**Exit criteria — degraded mode:**
- [ ] 1 controller rallentato (risposte a 25Hz invece di 50Hz) → gli altri 2 restano stabili
- [ ] Perdita telemetria debug (DIAG_HOLD/RPROBE) → control loop non impattato
- [ ] Restart parziale di 1 controller durante operazione → il Jetson isola il fault e gli altri 2 continuano
- [ ] Host jitter temporaneo (50ms spike) → nessun watchdog timeout sui controller
- [ ] Bus error burst (10 frame corrotti) → recovery automatico senza intervento umano

---

## L6 — Half Leg Assembly (Jetson)

**Stato:** 🔲

**Obiettivo:** Prima gamba completa assemblata meccanicamente. Validazione sotto carico reale.

**Piattaforma:** Jetson

**Pre-requisiti:**
- L3 (hip single) completato
- L5 (three-controller stress) completato
- Struttura meccanica assemblata (hip → knee → ankle)
- Cablaggio CAN definitivo (lunghezze, terminazioni)

**Cosa testa di nuovo:**
- Carico gravitazionale reale distribuito su 3 joint
- Interazione meccanica tra joint (muovere hip cambia carico su knee e ankle)
- Trajectory planning multi-joint: movimenti coordinati reali
- Cablaggio e connettori sotto stress meccanico
- Consumo energetico reale con tutti i motori sotto carico

**Test minimi:**
1. Init + homing sequenziale (hip → knee → ankle)
2. Hold stabile in postura neutra per 10 min
3. Movimento coordinato semplice: "piega la gamba" (knee flex + ankle dorsiflexion)
4. Movimento coordinato con carico: gamba appesa con peso
5. Recovery test: E-Stop durante movimento → safe stop
6. Sessione lunga: 30 min di movimenti misti

**Exit criteria:**
- [ ] Homing completo ripetibile (3/3 successi)
- [ ] Hold 10 min senza watchdog timeout o CAN error
- [ ] Movimento coordinato 3-joint fluido (no jerk tra fasi)
- [ ] E-Stop safe da qualsiasi stato
- [ ] Cablaggio: nessun contatto intermittente dopo 100 cicli di flessione
- [ ] Temperatura motori stabile dopo 15 min di attività mista
- [ ] Nessun degrado prestazioni in sessione lunga (30 min)

---

## L7 — Two Legs (Jetson)

**Stato:** 🔲

**Obiettivo:** Sistema completo a 12 DOF (6 joint, 12 motori).

**Piattaforma:** Jetson

**Pre-requisiti:**
- L6 (half leg) completato con successo
- Seconda gamba assemblata
- Bus CAN: valutare se servono 2 bus separati (1 per gamba) o 1 bus basta

**Domanda architetturale aperta: topologia CAN**

| Opzione | Pro | Contro |
|---------|-----|--------|
| 1 bus host | Semplice, meno hardware | 960 frame/s, ~23% bandwidth — fattibile |
| 2 bus host (L/R) | Isolamento fault, bandwidth doppia | Serve seconda interfaccia CAN su Jetson |

Decisione rimandata a dopo i dati di L5. Non scegliere per prudenza astratta — misurare prima.

**Test minimi:**
1. Init entrambe le gambe sequenzialmente
2. Hold bilaterale simmetrico
3. Movimento specular (entrambe le gambe fanno lo stesso)
4. Movimento indipendente (una gamba si muove, l'altra in hold)
5. Stress test: movimenti continui alternati per 60 min
6. Fault injection: scollegare un controller e verificare che l'altra gamba resta safe

**Exit criteria:**
- [ ] Init 6 controller ripetibile
- [ ] Hold bilaterale 15 min senza errori
- [ ] Movimento coordinato 12 DOF fluido
- [ ] Fault isolation: perdita di 1 controller non impatta l'altra gamba
- [ ] Sessione lunga 60 min senza degrado
- [ ] CAN bus utilization documentata e sotto 60%

---

## Riepilogo Timeline

```
L1    Knee single        ✅  Closed (baseline)
L2    Ankle single       ✅  Closed (baseline)
L3    Hip single         ⏸  Blocked by hybrid model
L3.5  Protocol freeze    🔲  Next documentation gate
L4    2-ctrl bench       🟡  Bench-validated, formal close after L3.5
L5    3-ctrl stress      🔲  Dopo L4
L6    Half leg           🔲  Dopo L5
L7    Two legs           🔲  Dopo L6
```

**Parallelismo possibile:**
- L3 (HIP) è bloccato da refactor architetturale, non da disponibilità CAN/Jetson
- L3.5 può partire subito come lavoro documentale/protocol freeze
- L4 ha già evidenza di banco, ma non va "chiuso" formalmente senza L3.5
- L5 richiede:
  - L4 consolidato
  - oppure decisione esplicita di usare l'attuale protocollo come baseline temporanea per stress test

---

## Prossimi Passi Raccomandati (2026-03-31)

Ordine raccomandato, dato lo stato reale del banco:

1. **Congelare `KNEE_RIGHT + ANKLE_RIGHT` come regression bench Jetson**
   - non usare più questo banco per refactor esplorativi
   - usarlo per verificare che i cambi futuri non rompano:
     - startup nominale
     - home / nudge manuali
     - exercise automatico
     - recovery post-`E-stop` nella stessa sessione Jetson

2. **Chiudere `L3.5 Protocol Freeze`**
   - l'evidenza raccolta su `knee + ankle` è già sufficiente per congelare:
     - cadence comandi
     - watchdog policy
     - path startup nominale
     - path recovery same-session post-`E-stop`
     - set minimo di telemetria production
   - questo è il prossimo gate documentale reale

3. **Aprire il refactor `HIP` come binario separato**
   - non usare il banco `knee + ankle` per "simulare" la `HIP`
   - implementare invece il modello ibrido come da [HIP_HYBRID_DOF_SPEC.md](software/docs/HIP_HYBRID_DOF_SPEC.md)

4. **Solo dopo il refactor `HIP`, fare il primo bring-up `HIP` singolo**
   - non prima
   - il rischio dominante ora non è più CAN/Jetson: è il modello software sbagliato del joint ibrido

5. **Rimandare il caso "Jetson restart after E-stop" a bugfix successivo**
   - utile, ma non blocca la milestone attuale
   - non deve ritardare `L3.5` o il refactor `HIP`

---

## Appendice: CAN Rate Profile Reference

Per facilitare il dimensionamento a ogni livello.

**Per singolo joint (1 controller):**

| Stream | Direction | Rate | Frames |
|--------|-----------|------|--------|
| Impedance command | Host→Controller | 50 Hz | 1 |
| Torque cmd+resp (per motor) | Controller↔Motor | 50 Hz | 2 × N_motors |
| Encoder stream | Controller→Host | 50 Hz | 1 |
| JOINT_STATE | Controller→Host | ~10 Hz | 1 |
| DIAG_HOLD | Controller→Host | 0.3 Hz | 2 |
| RPROBE | Controller→Host | on-demand | 1 |

**Bus host (Host↔Controller) per N joint:**
- Command: N × 50 = frame/s
- Encoder: N × 50 = frame/s
- JOINT_STATE: N × 10 = frame/s
- Telemetria: ~N × 1 = frame/s
- **Totale: ~111 × N frame/s**

| Config | N | Host bus frame/s | % of ~3570 practical max |
|--------|---|-----------------|--------------------------|
| Knee | 1 | ~111 | ~3% |
| Knee+Ankle | 2 | ~222 | ~6% |
| Half leg | 3 | ~333 | ~9% |
| Two legs | 6 | ~666 | ~19% |

Stime conservative (ordine di grandezza). Il bus fisico probabilmente regge;
il collo di bottiglia sarà il software host. Da verificare con test strumentato (L5).
