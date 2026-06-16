FROM python:3.12-slim

WORKDIR /srv

# Install deps first for layer caching.
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app ./app

# Local-mirror files live here (mount a volume to populate). vtrd needs no volume.
ENV CATALOG_SITES=local,vtrd \
    CATALOG_LOCAL_DIR=/data/files
RUN mkdir -p /data/files

EXPOSE 8080
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8080"]
