## Teamcode-Modul

Willkommen zu unserem Team-Code-Modul. Hier haben wir Code. Wenn du aus einem zukünftigen Jahr der Team 8097 Software kommst, würdest du wahrscheinlich noch ein Jahr auf Ryans Code zurückgehen, was wesentlich besser ist als bei uns. Danke fürs Lesen.

## Erstellen Sie Ihre eigenen OpModes

Die einfachste Möglichkeit, einen eigenen OpMode zu erstellen, besteht darin, einen Sample-OpMode zu kopieren und ihn zu einem eigenen zu machen.

Beispielopmodes sind im FtcRobotController-Modul vorhanden.
Um diese Beispiele zu finden, suchen Sie das Modul FtcRobotController auf der Registerkarte "Projekt / Android".

Erweitern Sie die folgenden Baumelemente:
 FtcRobotController / java / org.firstihinspires.ftc.robotcontroller / externe / Proben

In diesem Ordner kann eine Reihe verschiedener Beispielklassen angezeigt werden.
Die Klassennamen folgen einer Namenskonvention, die den Zweck jeder Klasse angibt.
Die vollständige Beschreibung dieser Konvention finden Sie in der Datei samples / sample_convention.md.

Eine kurze Zusammenfassung der Namenskonvention finden Sie hier:
Der Präfix des Namens ist einer der folgenden:

* Basic: Dies ist ein minimal funktionaler OpMode zur Veranschaulichung des Skeletts / der Struktur
            eines bestimmten Stils von OpMode. Dies sind Nackten Knochen Beispiele.
* Sensor: Dies ist ein Sample-OpMode, der zeigt, wie ein bestimmter Sensor verwendet wird.
            Es ist nicht als funktionierender Roboter gedacht, es zeigt lediglich den minimalen Code
            erforderlich, um die Sensorwerte zu lesen und anzuzeigen.
* Hardware: Dies ist kein tatsächlicher OpMode, sondern eine Hilfsklasse, die zur Beschreibung verwendet wird
            die Hardware eines bestimmten Roboters: zB: für einen Pushbot. Sieh dir irgendwas an
            Pushbot-Beispiel, um zu sehen, wie dies in einem OpMode verwendet werden kann.
            Teams können eine davon kopieren, um ihre eigene Roboterdefinition zu erstellen.
* Pushbot: Dies ist ein Sample-OpMode, der die Pushbot-Roboterstruktur als Basis verwendet.
* Konzept: Dies ist ein Beispiel-OpMode, der die Ausführung einer bestimmten Funktion oder eines bestimmten Konzepts veranschaulicht.
            Diese können komplex sein, aber ihre Funktionsweise sollte in den Kommentaren klar erläutert werden,
            oder die Kopfzeile sollte auf ein externes Dokument, eine Anleitung oder ein Tutorial verweisen.
* Bibliothek: Dies ist eine Klasse oder eine Gruppe von Klassen, die zur Implementierung einer Strategie verwendet werden.
            Diese implementieren normalerweise keinen vollständigen OpMode. Stattdessen werden sie einbezogen
            durch einen OpMode, um einige Stand-alone-Fähigkeiten bereitzustellen.

Sobald Sie sich mit der Auswahl der verfügbaren Muster vertraut gemacht haben, können Sie eines davon auswählen
Grundlage für Ihren eigenen Roboter. In allen Fällen muss das gewünschte Sample kopiert werden
Ihr TeamCode-Modul wird verwendet.

Dies geschieht direkt in Android Studio mit den folgenden Schritten:

 1) Suchen Sie die gewünschte Beispielklasse in der Projekt / Android-Struktur.

 2) Klicken Sie mit der rechten Maustaste auf die Beispielklasse und wählen Sie "Kopieren"

 3) Erweitern Sie den Ordner TeamCode / java

 4) Klicken Sie mit der rechten Maustaste auf den Ordner org.firstinspires.ftc.teamcode und wählen Sie "Einfügen"

 5) Sie werden nach einem Klassennamen für die Kopie gefragt.
    Wählen Sie basierend auf dem Zweck dieser Klasse etwas Sinnvolles.
    Beginnen Sie mit einem Großbuchstaben und denken Sie daran, dass später möglicherweise ähnliche Klassen vorhanden sind.

Sobald Ihre Kopie erstellt wurde, sollten Sie sie für den Einsatz auf Ihrem Roboter vorbereiten.
Dies geschieht, indem der Name des OpModes angepasst und auf dem angezeigt wird
Die OpMode-Liste der Treiberstation.

Jede OpMode-Beispielklasse beginnt mit mehreren Codezeilen wie die folgenden:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

Der Name, der auf der "opmode list" der Treiberstation erscheint, wird durch den Code definiert:
 ``name="Template: Linear OpMode"``
Sie können das, was zwischen den Anführungszeichen steht, ändern, um Ihren Opmode besser zu beschreiben.
Der "group =" - Teil des Codes kann verwendet werden, um bei der Organisation Ihrer OpModes-Liste zu helfen.

Wie gezeigt, wird der aktuelle OpMode NICHT auf der OpMode-Liste der Treiberstation angezeigt
  ``@Disabled`` Anmerkung, die enthalten ist.
Diese Zeile kann einfach gelöscht oder auskommentiert werden, um den OpMode sichtbar zu machen.



## ADVANCED Multi-Team App-Verwaltung: Klonen des TeamCode-Moduls

In einigen Situationen haben Sie mehrere Teams in Ihrem Club und möchten, dass sie alle teilen
eine gemeinsame Code-Organisation, wobei jeder in der Lage ist, * den * anderen Code * zu sehen, aber jeder hat
ein eigenes Teammodul mit eigenem Code, den sie selbst pflegen.

In diesem Fall möchten Sie möglicherweise das TeamCode-Modul einmal für jedes dieser Teams klonen.
Jeder der Klone würde dann nebeneinander in der Android Studio-Modulliste erscheinen,
zusammen mit dem FtcRobotController-Modul (und dem ursprünglichen TeamCode-Modul).

Selektive Team-Telefone können dann programmiert werden, indem das gewünschte Modul aus der Pulldown-Liste ausgewählt wird
vor dem Klicken auf den grünen Run-Pfeil.

Warnung: Dies ist nicht für den unerfahrenen Softwareentwickler.
Sie müssen mit den Dateimanipulationen und der Verwaltung von Android Studio-Modulen vertraut sein.
Diese Änderungen werden AUSSERHALB von Android Studios durchgeführt, also schließen Sie Android Studios, bevor Sie dies tun.
 
Auch .. Machen Sie eine vollständige Projekt-Backup, bevor Sie das beginnen :)

Um TeamCode zu klonen, gehen Sie folgendermaßen vor:

Hinweis: Einige Namen beginnen mit "Team" und andere beginnen mit "Team". Dies ist beabsichtigt.

1) Kopieren Sie den gesamten "TeamCode" mit den Dateiverwaltungstools Ihres Betriebssystems
    Ordner zu einem Geschwisterordner mit einem entsprechenden neuen Namen, zB: "Team0417".

2) Löschen Sie im neuen Ordner Team0417 die Datei TeamCode.iml.

3) den neuen Ordner Team0417, benennen Sie den Ordner "src / main / java / org / fireinspires / ftc / teamcode" um
    zu einem passenden Namen mit einem Kleinbuchstaben "Team" zB: "team0417".

4) Bearbeiten Sie im neuen Ordner Team0417 / src / main die Datei "AndroidManifest.xml", und ändern Sie die Zeile, die enthält
         package = "org.firstinspires.ftc.teamcode"
    sein
         package = "org.firstinspires.ftc.team0417"

5) Hinzufügen: Fügen Sie ': Team0417' in die Datei "/settings.gradle" ein.
    
6) Öffnen Sie Android Studios und bereinigen Sie alle alten Dateien, indem Sie das Menü "Build / Clean Project" verwenden. "
