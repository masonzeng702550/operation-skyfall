FROM python:3.11-slim

RUN pip install --no-cache-dir pymavlink aiohttp

WORKDIR /opt
COPY flag_watcher.py /opt/flag_watcher.py

EXPOSE 9000
CMD ["python3", "-u", "/opt/flag_watcher.py"]
