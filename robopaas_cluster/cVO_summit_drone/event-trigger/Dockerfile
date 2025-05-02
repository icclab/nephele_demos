# Use a minimal Python image
FROM python:3.9-slim

# Set the working directory
WORKDIR /app

# Copy the script into the container
COPY trigger_alert.py /app/

# Expose port 8000 (Prometheus will scrape metrics from here)
EXPOSE 8000

# Run the Python script
CMD ["python", "/app/trigger_alert.py"]

