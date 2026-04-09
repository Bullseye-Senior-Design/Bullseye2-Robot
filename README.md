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
Description=Bullseye Startup Program
After=network.target

[Service]
ExecStart=/home/bullseye/Desktop/Bullseye2-Robot/.venv/bin/python /home/bullseye/Desktop/Bullseye2-Robot/main.py
WorkingDirectory=/home/bullseye/Desktop/Bullseye2-Robot
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
sudo systemctl enable bullseye_boot.service
```

To see if the service is operational run:
```bash
sudo systemctl status bullseye_boot.service
```

To monitor the terminal output run:
```bash
sudo journalctl -u bullseye_boot.service -f
```

To restart the service run:
```bash
sudo systemctl restart bullseye_boot.service
```

To stop the service run:
```bash
sudo systemctl stop bullseye_boot.service
```