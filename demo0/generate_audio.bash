#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
AUDIO_DIR="${ROOT_DIR}/src/socialbot_demo0/assets/audio"
mkdir -p "$AUDIO_DIR"

if command -v espeak-ng >/dev/null 2>&1; then
    ESPEAK=espeak-ng
elif command -v espeak >/dev/null 2>&1; then
    ESPEAK=espeak
else
    echo "Install espeak-ng or run this script inside the Demo 0 container." >&2
    exit 1
fi

"$ESPEAK" -v it -s 145 -p 48 -w "$AUDIO_DIR/introduction.wav" \
    "Ciao! Sono il robot della dimostrazione. Vorrei farti due brevi domande sulla robotica educativa."
"$ESPEAK" -v it -s 140 -p 50 -w "$AUDIO_DIR/question_01.wav" \
    "Prima domanda. Quale vantaggio pensi possa offrire un robot educativo alle persone?"
"$ESPEAK" -v it -s 150 -p 50 -w "$AUDIO_DIR/acknowledgement_01.wav" \
    "Grazie. Passiamo alla seconda domanda."
"$ESPEAK" -v it -s 140 -p 50 -w "$AUDIO_DIR/question_02.wav" \
    "Seconda domanda. In quale situazione ti piacerebbe interagire con un robot come me?"
"$ESPEAK" -v it -s 145 -p 48 -w "$AUDIO_DIR/conclusion.wav" \
    "Grazie per il tuo tempo. L'intervista è terminata e ora torno alla posizione iniziale."
"$ESPEAK" -v it -s 145 -p 45 -w "$AUDIO_DIR/navigation_failed.wav" \
    "Non sono riuscito a completare la missione. È necessario controllare la navigazione."
"$ESPEAK" -v it -s 150 -p 30 -w "$AUDIO_DIR/scripted_answer_01.wav" \
    "Penso che possa spiegare concetti complessi in modo più coinvolgente e interattivo."
"$ESPEAK" -v it -s 150 -p 30 -w "$AUDIO_DIR/scripted_answer_02.wav" \
    "Mi piacerebbe incontrarlo in un museo, in una scuola oppure durante una fiera scientifica."

echo "Audio files generated in ${AUDIO_DIR}"
