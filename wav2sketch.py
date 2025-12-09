import os
import struct
import sys
import wave

def print_usage():
    print("Usage: python wav2sketch.py [directory]")
    print("  directory: Path to folder containing .wav files (default: current directory)")

def convert_wav_to_c(filepath, output_h, output_cpp):
    filename = os.path.basename(filepath)
    name, ext = os.path.splitext(filename)
    
    # Clean variable name (only alphanumeric)
    var_name = "AudioSample" + "".join(x for x in name.title() if x.isalnum())
    
    try:
        with wave.open(filepath, 'rb') as wav:
            # Verify format
            if wav.getnchannels() != 1: # Stereo to Mono could be added, but for now strict
                 print(f"Skipping {filename}: Must be Mono")
                 return None
            if wav.getsampwidth() != 2: # 16-bit
                 print(f"Skipping {filename}: Must be 16-bit")
                 return None
            if wav.getframerate() != 44100:
                 print(f"Warning {filename}: Rate is {wav.getframerate()}, expected 44100")
            
            nframes = wav.getnframes()
            data = wav.readframes(nframes)
            
            # Convert bytes to audio_block_samples (uint32 padding) behavior if needed uses ula2?
            # Actually standard simple wav2sketch just dumps the PCM data with some padding
            # The format expected by AudioplayMemory is raw PCM data aligned to 32 bits usually? 
            # No, AudioPlayMemory expects an array where the first 2 ints are length and format (optional? no, standard assumes just data usually or length encoded)
            # checking PJRC source: It requires the array to allow proper access.
            # actually AudioPlayMemory takes `const unsigned int *`.
            # First 32-bit int is length (in samples? bytes? check docs).
            # Usually strict: AudioPlayMemory::play(const unsigned int *data)
            # data[0] is ignored? No.
            # looking at generated files from standard wav2sketch:
            # const unsigned int AudioSampleName[SIZE] = {
            #   0x81000000 | (length_in_bytes), ... data ...
            # } 
            # Actually the format is:
            # [0] = length in bytes | 0x81000000 (if u-law) or just length? 
            # Wait, AudioPlayMemory usually plays raw 16-bit PCM.
            # Let's look at a reference. 
            # "The array must begin with the length in bytes (including the header itself? no)."
            # Actually widely used `wav2sketch` produces:
            #  unsigned int AudioSampleX[] = {
            #    0x2000000 | length_bytes, ... (format flags)
            #  }
            
            # SIMPLIFICATION:
            # We will generate standard 16-bit PCM arrays padded to 4-byte boundaries.
            # The first word is the length in samples? Or bytes?
            # It seems AudioPlayMemory uses "length in bytes".
            # and often OR'd with logical flags. 
            
            # Let's stick to the simplest working format for AudioPlayMemory:
            # It just plays raw data if we aren't careful? No it needs a header word.
            # Header: (length_in_bytes) -- but often needs encoding flags.
            # For 16-bit PCM, it is effectively: length | 0x81000000 (if uLaw? no).
            # 
            # Let's use the widely accepted format:
            # Header = length_bytes | 0x01000000 (if PCM16? No).
            
            # Let's try to replicate the official C code logic in Python.
            # Official wav2sketch.c:
            # header = length | 0x00000000; (for PCM16)
            # ... audio_block_samples ...
            
            # Actually, standard AudioPlayMemory just expects the raw data pointer for `play(data)`.
            # Wait, looking at `AudioPlayMemory.cpp`:
            # void AudioPlayMemory::play(const unsigned int *data) { ...
            # uint32_t header = *data;
            # uint32_t length = header & 0x00FFFFFF;
            # uint32_t format = header & 0xFF000000;
            # ...
            # Format: 0x00 = PCM16, 0x81 = u-law ...
            
            # So Header = length_in_bytes. (PCM16 is 0).
            # Data follows immediately. 
            # But the data array is `unsigned int` (32-bit). 
            # So 2 16-bit samples are packed into 1 32-bit int.
            # Order: LSB first? 
            # Sample 0 is lower 16 bits, Sample 1 is upper 16 bits.
            
            values = []
            # Header
            # Length should be total bytes of audio data.
            total_bytes = nframes * 2
            values.append(total_bytes) 
            
            # Unpack and repack
            # Read two samples at a time
            shorts = struct.unpack(f"<{nframes}h", data)
            
            for i in range(0, len(shorts), 2):
                s1 = shorts[i] & 0xFFFF
                s2 = shorts[i+1] & 0xFFFF if i+1 < len(shorts) else 0
                packed = (s2 << 16) | s1
                values.append(packed)
            
            output_h.write(f"extern const unsigned int {var_name}[{len(values)}];\n")
            
            output_cpp.write(f"// {filename}: {nframes} frames, {total_bytes} bytes\n")
            output_cpp.write(f"const unsigned int {var_name}[{len(values)}] = " + "{\n")
            
            # Write hex data
            for i, val in enumerate(values):
                if i % 8 == 0: output_cpp.write("  ")
                output_cpp.write(f"0x{val:08X},")
                if i % 8 == 7: output_cpp.write("\n")
            
            output_cpp.write("\n};\n\n")
            
            return var_name

    except Exception as e:
        print(f"Error processing {filename}: {e}")
        return None

def main():
    target_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    
    h_file = open(os.path.join(target_dir, "AudioSample.h"), "w")
    cpp_file = open(os.path.join(target_dir, "AudioSample.cpp"), "w")
    
    h_file.write("#ifndef AUDIOSAMPLE_H\n#define AUDIOSAMPLE_H\n\n#include <Arduino.h>\n\n")
    cpp_file.write(f"#include \"AudioSample.h\"\n\n")
    
    found_any = False
    
    print(f"Scanning {target_dir} for .wav files...")
    
    for file in os.listdir(target_dir):
        if file.lower().endswith(".wav"):
            print(f"Converting {file}...")
            var_name = convert_wav_to_c(os.path.join(target_dir, file), h_file, cpp_file)
            if var_name:
                found_any = True
    
    h_file.write("\n#endif\n")
    
    h_file.close()
    cpp_file.close()
    
    if found_any:
        print("Done! Created AudioSample.h and AudioSample.cpp")
    else:
        print("No valid .wav files found.")

if __name__ == "__main__":
    main()
