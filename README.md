# arduino_code

This program obtains live ECG readings from a sensor, uses the Pan-Tompkins 
QRS-detection algorithm to calculate the corresponding Beats Per Minute (BPM), 
and stores the BPMs and their corresponding time stamps on the Arduino 101's 
2MB external flash memory chip. It also uses the 101's accelerometer to count 
steps, and it stores that data on the chip as well. When connected to the phone, 
it retrieves the BPMs and steps along with their time information from the chip 
and sends them to the phone live via a custom Bluetooth Low Energy service. It 
also sends live ECG measurements to the phone for a graph of the heart beat. 
When the phone is disconnected for a while and then reconnects, the device 
quickly sends all the accumulated BPM + time stamp data to the phone in bathces
and then resumes live updating.