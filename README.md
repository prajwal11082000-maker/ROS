# install Git on Ubuntu
### sudo apt update
### sudo apt install git -y
### git --version
### git config --global user.name "Your Name"
### git config --global user.email "your_email@example.com"
### git config --list


# Step-by-Step: Add “Open With SmartGit” Option:

### Make sure your .desktop entry exists
### nano ~/.local/share/applications/smartgit.desktop

### Paste (or update) this content:
[Desktop Entry]
Name=SmartGit
Comment=Git client for developers
Exec=/home/enmac/Downloads/smartgit/bin/smartgit.sh %F
Icon=/home/enmac/Downloads/smartgit/bin/smartgit.png
Type=Application
Categories=Development;IDE;
Terminal=false
MimeType=inode/directory;


### Make it executable
### chmod +x ~/.local/share/applications/smartgit.desktop


### Update desktop database
### update-desktop-database ~/.local/share/applications

### Restart the file manager (optional)
### nautilus -q

