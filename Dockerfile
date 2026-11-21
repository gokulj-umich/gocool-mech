# Gocool mech bot. Build: docker build -t gocool .
FROM python:3.13-slim
ENV OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
RUN pip install --no-cache-dir numpy==2.3.5
WORKDIR /bot
COPY mechfight.py bot.py navmap.npz ./
ENTRYPOINT ["python3", "-u", "/bot/bot.py"]
