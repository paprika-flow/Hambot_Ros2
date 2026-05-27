```
1. Write/Edit code in VS Code (Synced to the Pi)
                             │
                             ▼
  2. Compile instantly inside the running container (colcon build)
                             │
                             ▼
  3. Need a new package/tool? ──► Install it interactively to test:
                             │    "apt install ros-humble-xyz"
                             │
                             ▼
  4. Did it work? ──────────────► Add "ros-humble-xyz" to your Dockerfile,
                             │    but DO NOT rebuild yet. Keep coding!
                             │
                             ▼
  5. End of the day? ───────────► Run "docker compose build" to permanently
                                  bake the new tools into your image.
```
### Start of session
```bash
cd ~/robot_ws
docker compose up -d
docker exec -it hambot bash
```

### Adding new package

- add package in the interactive terminal
  - if it worked add it to the dockerfile so 
  - 

### End of session
```bash
docker compose down
docker compose build
```