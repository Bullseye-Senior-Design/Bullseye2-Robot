All programing should only have to be done in the Robot and transmission folders

# you will need to manually install the library to get gpio work on the venv. 
create with this:
```bash
python -m venv --system-site-packages .venv
    source .venv/bin/activate
```

In order to auto install required python Libraries run command below in the directory
```bash
    pip install -r requirements.txt
    sudo apt-get install python3-tk python3-pil python3-pil.imagetk 
```
(remove if adding python3-tk to requirements works)

# Pi Connection Info
login as: bullseye
password: pass0


# Creating the an Automatic bootup sequence
Create a systemd file using this command:
```bash
sudo nano /etc/systemd/system/bullseye_boot.service
```
Paste this into the file (Modify the file paths to be correct)
```txt
[Unit]
Description=My Python Startup Script
After=network.target

[Service]
ExecStart=/home/pi/myproject/.venv/bin/python /home/pi/myproject/myscript.py
WorkingDirectory=/home/pi/myproject/
StandardOutput=inherit
StandardError=inherit
Restart=always
User=bullseye

[Install]
WantedBy=multi-user.target
```
Then reload the sevice and enable it on boot
```bash
sudo systemctl daemon-reload
sudo systemctl enable myscript.service
```
To test the service is operational run:
```bash
sudo systemctl status myscript.service
```