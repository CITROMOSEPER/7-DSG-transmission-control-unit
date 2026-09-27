# 7 sebességes DSG automata váltó vezérlő (TCU)

Egy 7 fokozatú duplakuplungos (DSG) automata váltó vezérlőegységének megvalósítása **Schneider Zelio** PLC-n, egy hozzá épített **ESP32 alapú járműszimulátor-paddal**, OLED műszerfallal és **Arduino alapú motorhang-generátorral**.

Mivel egy valódi DSG váltó és motor több száz kilogramm, a PLC nem egy igazi hajtásláncot vezérel, hanem egy szimulátorpadot. Az ESP32 ugyanazokból a bemenetekből (gáz, fék, váltókar) dolgozik, mint a PLC, fizikai modell alapján kiszámolja a jármű sebességét és a motor fordulatszámát, majd ezeket analóg jelként visszaküldi a PLC-nek. A PLC ebből dönti el, melyik fokozatot kell kapcsolni és hogyan kell a két kuplungot mozgatni.

---

## Tartalom

- [Hogyan működik egy DSG?](#hogyan-működik-egy-dsg)
- [Rendszerfelépítés](#rendszerfelépítés)
- [Hardver](#hardver)
- [PLC I/O kiosztás](#plc-io-kiosztás)
- [A PLC program funkciói](#a-plc-program-funkciói)
- [Szimulátorpad (ESP32)](#szimulátorpad-esp32)
- [Motorhang-generátor (Arduino Nano)](#motorhang-generátor-arduino-nano)
- [Fájlok](#fájlok)
- [Fordítás és feltöltés](#fordítás-és-feltöltés)
- [Fejlesztési lehetőségek](#fejlesztési-lehetőségek)

---

## Hogyan működik egy DSG?

A duplakuplungos váltó lényegében két fél kézi váltó, amelyek párhuzamosan dolgoznak, mindegyik saját kuplunggal:

| Részváltó | Fokozatok | Kuplung |
| --- | --- | --- |
| Páratlan | 1, 3, 5, 7 | Kuplung 1 |
| Páros | 2, 4, 6, R | Kuplung 2 |

Mindkét részváltóban mindig kapcsolva van egy fokozat, de egyszerre csak az egyik kuplung zár. A másik oldalon a következő, várhatóan szükséges fokozat van **bekészítve** (előválasztva). Váltáskor az egyik kuplung nyit, a másik azonnal zár, így a nyomatékkimaradás akár 10 ms alatt marad. Ha a szükséges fokozat nem volt bekészítve (például hirtelen padlógáznál), mindkét kuplungot nyitni kell, ezért ilyenkor a váltás lassabb.

---

## Hardver

| Alkatrész | Szerep |
| --- | --- |
| Schneider Zelio **SR3B261BD** (24 V DC) | A váltóvezérlő (TCU) PLC. Digitális bemenetek I1...IA, relés kimenetek Q1...QA |
| Schneider Zelio **SR3XT43BD** | Analóg bővítő modul: 10 bites analóg bemenetek, 0-10 V-os analóg kimenetek (QB, QC) |
| **ESP32** (ESP-WROOM-32 devboard) | A jármű és a motor szimulációja, műszerfal kirajzolása |
| **Arduino Nano** | Motorhang generálása |
| 128x64 I2C OLED kijelző (SH1106 vagy SSD1306) | Műszerfal |
| DPST kapcsolók | Váltókar állások, kickdown, hiba |
| SPDT kapcsolók | Háromállású pillangóváltók (fel, le, túlhúzás) |
| Kétcsatornás lineáris potenciométerek | Gáz- és fékpedál (egyik csatorna a PLC-nek, másik az ESP32-nek) |
| 2 db 24 V-os szolenoid | Parkolózár és váltózár |
| LED-ek (zöld, sárga) | Visszajelzések, pl. kuplung pozíciók |
| Buzzer | Motorhang |
| 24 V-os C-sínes táp + USB táp | Tápellátás (közös hálózati vezetékről) |

A teljes kapcsolási rajz KiCad 9 formátumban: [`7DSG_support_circuit.kicad_sch`](7DSG_support_circuit.kicad_sch). A 24 V-os jeleket a rajz szintillesztéssel vezeti az ESP32 3,3 V-os bemeneteire, a PLC 0-10 V-os kuplung kimeneteit feszültségosztó és 3,3 V-os Zener dióda védi.

Az egész szerkezet egy cipősdobozban kapott helyet, az alkatrészek csavarokkal, anyákkal és gyorskötözővel vannak rögzítve.

---

## PLC I/O kiosztás

| Bemenet | Funkció | Kimenet | Funkció |
| --- | --- | --- | --- |
| I1 | P (DPST kapcsoló) | Q1 | Parkolózár (24 V-os szolenoid) |
| I2 | R (DPST kapcsoló) | Q2 | Váltózár (24 V-os szolenoid) |
| I3 | N (DPST kapcsoló) | Q3 | 1. fokozat (ESP32 bemenet) |
| I4 | D (DPST kapcsoló) | Q4 | 2. fokozat (ESP32 bemenet) |
| I5 | M (DPST kapcsoló) | Q5 | 3. fokozat (ESP32 bemenet) |
| I6 | LKES (SPDT kapcsoló #1) | Q6 | 4. fokozat (ESP32 bemenet) |
| I7 | Le (SPDT kapcsoló #1) | Q7 | 5. fokozat (ESP32 bemenet) |
| I8 | Fel (SPDT kapcsoló #2) | Q8 | 6. fokozat (ESP32 bemenet) |
| I9 | LNES (SPDT kapcsoló #2) | Q9 | 7. fokozat (ESP32 bemenet) |
| IA | Kickdown (DPST kapcsoló) | QA | Hátramenet (ESP32 bemenet) |
| IB | Gáz (kétcsatornás lin. poti) | QB | Kuplung 1 (ESP32 analóg bemenet) |
| IC | Fék (kétcsatornás lin. poti) | QC | Kuplung 2 (ESP32 analóg bemenet) |
| ID | Bemeneti RPM (ESP32 DAC) | | |
| IJ | Sebesség (ESP32 DAC) | | |
| IH | Hiba (DPST kapcsoló) | | |

**Rövidítések:** LKES = legkisebb elérhető sebességfokozat, KES = közepes elérhető sebességfokozat, LNES = legnagyobb elérhető sebességfokozat.

---

## A PLC program funkciói

A program a ZelioSoft 2 környezetben, funkcióblokkos (FBD) nyelven és SFC diagrammal készült. A programzóna 4096 bájtjából mindössze 9 bájt maradt szabadon. A program 14 funkcionális tömbre oszlik:

1. **Váltókar bemenetek.** A P, R, N, D, M állások 5 bemenetű ÉS makrókkal keresztreteszelik egymást, így egyszerre mindig csak egy mód lehet aktív.
2. **Pillangóváltók.** Háromállású pillangóváltók (a Koenigsegg Jesko megoldásához hasonlóan). Félig meghúzva és elengedve egy fokozatot vált, teljesen meghúzva a legkisebb (LKES) vagy legnagyobb (LNES) elérhető fokozatba ugrik. Mindkét fül teljes meghúzására mindkét kuplung nyit (átmeneti üres).
3. **Kickdown.** Padlógáz utáni „lerúgásra” azonnal a legkisebb elérhető fokozatba vált, M módban is, és amíg aktív, tiltja a pillangóváltókat. Hiba esetén nem elérhető.
4. **Analóg bemenetek.** A gáz és fék 8 bites (0-100 skála), a fordulatszám és sebesség a bővítő modul 10 bites bemenetein érkezik. Csak R, D vagy M módban engedődnek tovább (INTEN makró).
5. **Hiba bemenet.** Hibát szimulál a váltóban vagy motorban (vészüzem).
6. **Elérhető fokozatok keresése.** MIN-MAX ablakkomparátorok döntik el, hogy adott sebességnél melyik fokozat használható (se túlpörgés, se lefulladás). Ebből az LKES, KES és LNES makrók számolják a legkisebb, közepes és legnagyobb elérhető fokozatot.
7. **Gázállás kategorizálás.** Alacsony (0-30%), közepes és magas (71-100%) kategóriák.
8. **Duplapedál tiltás.** Ha mindkét pedál 50% felett van és a sebesség nagyobb, mint 50 km/h, mindkét kuplung nyit, amíg a helyzet meg nem szűnik.
9. **Launch control (rajtautomatika).** Álló helyzetben, D vagy M módban fék és gáz padlóra, kb. 5000 RPM körül aktiválódik: előfeszíti az 1-es kuplungot, a fék elengedésére az autó „kilő”. A gáz elengedése megszakítja.
10. **Automata (D) és manuál (M) fokozatválasztó.** D módban alacsony gáznál LNES, közepesnél KES, magasnál LKES kerül kiválasztásra. M módban csak a sofőr vált, kivéve ha a fordulatszám 800 alá esik (kényszer-visszaváltás).
11. **Váltás folyamata.** Háromlépéses SFC: kuplungok nyitása --> előválasztott fokozat beírása a kapcsolt fokozat regiszterébe --> a megfelelő kuplung zárása. Akkor indul, ha a cél fokozat eltér a kapcsolttól.
12. **Kuplungok kezelése.** A KGMOD makró végzi az elindulási modulációt 1-es fokozatban és hátramenetben (750 RPM alatt nyit, 800 RPM felett zár, 5 km/h felett teljesen zár). A KPLNG makró tárolja a kuplung értékét, és 0,1 s-os ütemben változtatja. Hiba esetén a kuplung legfeljebb 70%-ig zárhat.
13. **Fokozatok kapcsolása.** Két szó-bit átalakító tárolja a kapcsolt és az előválasztott fokozatot. Az előválasztott fokozat eggyel nagyobb, de 1500 RPM alatt vagy 20% feletti fékállásnál eggyel kisebb. 1-esben mindig 2-es, 7-esben mindig 6-os van bekészítve.
14. **Kimenetek kezelése.** A PARK makró 30 km/h alatt engedi a parkolózárat (véd a zárópöcök letörése ellen). A VLTZR makró mozgás közben zárja a váltókart, és P vagy N módból csak fékre lépve engedi ki. A D és M közötti váltás mindig szabad.

---

## Szimulátorpad (ESP32)

Forrás: [`7DSG_support.ino`](7DSG_support.ino)

Az ESP32 a PLC kimeneteiből (bekapcsolt fokozatok, két kuplung értéke) és a kezelőszervekből egy egyszerű, de fizikailag megalapozott járműmodellt futtat.

**Járműmodell (20 ms-os lépésköz):**

- Newton II. törvénye alapján: motorerő, légellenállás, gördülési ellenállás, emelkedő és fékerő (regeneratív és súrlódásos arányban)
- Alapértelmezett paraméterek: 1800 kg, Cd = 0,24, 2,2 m² homlokfelület, 0,33 m kerék sugár, 700 RPM lefulladás, 8000 RPM piros tartomány
- Fokozatonkénti sebességtartományok és nyomatékszorzók
- Kuplungcsúszás modellezése: a fordulatszám a szabadon pörgő és a kerekekhez kötött érték között keveredik a kuplung zártságától függően
- Nyomatékgörbe és nyomatékcsökkentés a piros tartomány közelében, 300 km/h-s sebességkorlátozó
- Launch control: a motor 6000 RPM körül tart, amíg a fék le van nyomva
- Kickdown: plusz teljesítmény („BOOST”)
- Hiba mód: gáz 30%-ra, motorerő csökkentve (vészüzem)

**Kimenetek a PLC felé:** a sebesség és a fordulatszám a két DAC lábon (0-255) megy vissza a PLC analóg bemeneteire, a kuplung értékek PWM-mel LED-eken is megjelennek.

**Műszerfal (128x64 OLED, U8g2):** a kirajzolás FreeRTOS taszkként a 0-s magon fut, a fizika az 1-es magon. Kijelzi a módot (P, R, N, D, M), a kapcsolt fokozatot a szomszédos elérhető fokozatokkal, a fordulatszámot mutatós órával és számmal, a sebességet, a két kuplung, a gáz és a fék állását oszlopokkal, valamint villogó „BOOST” és „ERR” jelzéseket.

---

## Motorhang-generátor (Arduino Nano)

Forrás: [`Engine_sound_generator.ino`](Engine_sound_generator.ino)

A Nano a szimulált fordulatszámból, gáz- és fékállásból valós időben állít elő motorhangot egy buzzeren:

- Gyújtási események szimulálása a 720°-os munkaciklusban, konfigurálható hengerszámmal, elrendezéssel (`I` vagy `V`), hengersor-szöggel és főtengely-típussal (`FLAT` vagy `CROSS`). Alapértelmezés: V6, flat-plane.
- A hangmagasság a fordulatszámmal arányos.
- **Launch mód** (kb. 6000 RPM, gáz és fék egyszerre): véletlenszerű gyújtáskimaradásos, „géppuska” szerű pattogás.
- **Hiba mód:** egyenetlen, cross-plane jellegű járás.
- **Kickdown:** +500 RPM-es hangmagasság-emelés.

---

## Fájlok

```
.
├── 7DSG_support.ino                                                 # ESP32: járműszimuláció + OLED műszerfal
├── Engine_sound_generator.ino                                       # Arduino Nano: motorhang-generátor
├── 7DSG_support_circuit.kicad_sch                                   # KiCad 9 kapcsolási rajz
└── ND_PROJEKTMUNKA_-_7_sebességes_DSG_automata_váltó_vezérlő.docx   # Részletes projektdokumentáció (PLC program ábrákkal)
```

---

## Fordítás és feltöltés

### ESP32 (`7DSG_support.ino`)

1. Arduino IDE-ben telepítsd az **ESP32 by Espressif** board csomagot (3.x verzió, mivel a kód a `ledcAttachChannel` API-t használja).
2. Könyvtárkezelőből telepítsd az **U8g2** könyvtárat.
3. Ha SSD1306 kijelződ van, a fájl elején cseréld ki a kijelző konstruktort (a kommentben megtalálod).
4. Válaszd ki az ESP32 Dev Module boardot, és töltsd fel.

### Arduino Nano (`Engine_sound_generator.ino`)

1. Külső könyvtár nem szükséges.
2. A motor karakterét a fájl elején lehet állítani: `CYLINDERS`, `LAYOUT`, `BANK_ANGLE_DEG`, `CRANK_TYPE`.
3. Válaszd ki a Nano boardot, és töltsd fel.

### PLC

A váltóvezérlő program a Schneider **Zelio Soft 2** szoftverrel készült SR3B261BD + SR3XT43BD konfigurációra. A program részletes, ábrákkal illusztrált leírása a projektdokumentációban található.

---

## Fejlesztési lehetőségek

A PLC programmemóriája gyakorlatilag betelt, ezért a következő ötletek nem fértek bele:

- **Kormányszög alapú kuplungzárás:** nagy kormányszögnél lágyabb kuplungzárás váltáskor, hogy kanyarban ne legyen váratlan tapadásvesztés.
- **Manuális kuplung-felülírás:** F1-es mintára egy kormányon lévő kezelőszervvel közvetlenül lehetne kezelni a kuplungot elinduláskor és launch controlnál, a vezérlő által számolt biztonságos határra korlátozva (lefulladás ellen).

---

## Szerző

**Nemes Dániel**
