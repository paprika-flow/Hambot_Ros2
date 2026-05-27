#!/bin/bash

# Read current version from the .env file if it exists
if [ -f .env ]; then
  CURRENT_VERSION=$(grep '^ROBOT_IMAGE_TAG=' .env | cut -d'=' -f2)
else
  CURRENT_VERSION="1.0.0" # Default fallback if no .env file exists yet
fi

echo "Current SIM Image Version: $CURRENT_VERSION"

# Prompt the user for the new version
read -p "Enter new version (Press Enter to keep $CURRENT_VERSION): " NEW_VERSION

# If the user pressed Enter without typing anything, keep the current version
if [ -z "$NEW_VERSION" ]; then
  NEW_VERSION=$CURRENT_VERSION
fi

# Update the .env file 
if [ -f .env ]; then
  # Strip out the old version line, write the rest to a temp file, and append the new version
  grep -v '^ROBOT_IMAGE_TAG=' .env > .env.tmp
  echo "ROBOT_IMAGE_TAG=$NEW_VERSION" >> .env.tmp
  mv .env.tmp .env
else
  # If .env does not exist, create it with the new version
  echo "ROBOT_IMAGE_TAG=$NEW_VERSION" > .env
fi

echo "Saved version $NEW_VERSION to .env file."
echo "Starting Docker build..."
echo ""

# Run the Docker Compose build
docker compose build