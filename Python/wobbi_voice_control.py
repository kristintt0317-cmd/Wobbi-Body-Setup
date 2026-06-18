import time
import serial
import speech_recognition as sr

# Change this to your Arduino COM port
ARDUINO_PORT = "COM5"
BAUDRATE = 9600

# Voice command mapping
COMMANDS = {
    "stop": "n",   # neutral pose
    "let's go": "r",     # repeat gait loop
    "hello": "g",  # one gait cycle
}

def send_command(ser, command_char):
    ser.write(command_char.encode("utf-8"))
    ser.flush()
    print(f"Sent to Arduino: {command_char}")

def main():
    recognizer = sr.Recognizer()

    print("Connecting to Arduino...")
    ser = serial.Serial(ARDUINO_PORT, BAUDRATE, timeout=1)
    time.sleep(2)

    print("Voice control ready.")
    print('Say: "stop", "go", or "hello"')
    print("Press Ctrl+C to quit.")

    with sr.Microphone() as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)

        while True:
            try:
                print("\nListening...")
                audio = recognizer.listen(source, timeout=5, phrase_time_limit=3)

                print("Recognizing...")
                text = recognizer.recognize_google(audio).lower().strip()
                print(f"Heard: {text}")

                for voice_word, arduino_cmd in COMMANDS.items():
                    if voice_word in text:
                        send_command(ser, arduino_cmd)
                        break
                else:
                    print("No valid command recognized.")

            except sr.WaitTimeoutError:
                print("No speech detected.")
            except sr.UnknownValueError:
                print("Could not understand.")
            except sr.RequestError as e:
                print(f"Speech recognition error: {e}")
            except KeyboardInterrupt:
                print("\nExiting voice control.")
                send_command(ser, "n")
                break

    ser.close()

if __name__ == "__main__":
    main()