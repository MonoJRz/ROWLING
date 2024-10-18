import subprocess

# Path to the Python script you want to run
script_path = r'C:\Users\BARTLAB\Desktop\Navi_server\Final\server.py'

# Command to run the script using Python
command = ['python', script_path]

# Using subprocess to execute the script
process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# Optional: Wait for the process to complete
stdout, stderr = process.communicate()

# Check if there are any errors
if process.returncode != 0:
    print("Error running script:")
    print(stderr.decode())
else:
    print("Script output:")
    print(stdout.decode())
