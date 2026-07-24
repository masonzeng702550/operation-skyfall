FROM python:3.11-slim

RUN pip install --no-cache-dir pymavlink

WORKDIR /opt
COPY relay.py /opt/relay.py

EXPOSE 14580
CMD ["python3", "-u", "/opt/relay.py"]
