FROM python:3.11-slim

RUN pip install --no-cache-dir pymavlink aiohttp

WORKDIR /opt
COPY app.py /opt/app.py
COPY static /opt/static

EXPOSE 8080
CMD ["python3", "-u", "/opt/app.py"]
