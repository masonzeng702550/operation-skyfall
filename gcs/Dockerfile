FROM python:3.11-slim

RUN pip install --no-cache-dir pymavlink

WORKDIR /opt
COPY gcs.py /opt/gcs.py
COPY mission.txt /opt/mission.txt

CMD ["python3", "-u", "/opt/gcs.py"]
