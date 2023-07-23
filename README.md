# coding_challenge_buzzer

Ich habe mich dafür entschieden den Ansatz zu implementieren, den ich in meinem letzten STM-Projekt verfolgt habe.
Hierfür habe ich einen Timer (TIM1) so konfiguriert, dass dieser eine Periodendauer von einer Mikrosekunde hat.
So kann ich die Periodendauer von der Noten-Frequenz berechnen und den Pin so lange mit dem doppelten Wert (rechteckige "Sinuswelle") ein und ausschalten.
Dies wird so lange ausgeführt, bis ein zweiter Timer (TIM2), welcher die gewünschte Dauer in Millisekunden hochzählt über den Compare-Wert läuft.
Ich habe des Weiteren angenommen, dass dem Buzzer die Spannung in eine Richtung reicht um einen Ton zu generieren.
Ansonsten könnte die H-Brücke des Treibers verwendet werden um die Stromflussrichtung umzukehren und nicht nur den Stromfluss zu deaktivieren.
Des Weiteren gehe ich davon aus, dass die Polarität des Buzzers irrelevant ist.

Der Ansatz den ich gewählt habe ist wahrscheinlich nicht der effektivste.
Es wäre sinnvoller einen Timer so zu konfigurieren, dass ein PWM-Signal generiert wird und die Frequenz von diesem variiert wird.
Ich habe mich gegen diesen Ansatz entschieden, da dort der Timer in einer hohen Frequenz neu initiiert werden müsste.
Es war für mich vor allem eine Herausforderung den Code nicht ausführen zu können.
Normalerweise würde ich die einzelnen Komponenten anhand der Hardware mit meinem Oszi an meiner Seite testen.