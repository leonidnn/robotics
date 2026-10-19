FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

# Тесты + все численные результаты отчета (печатаются и сохраняются в results/)
CMD ["sh", "-c", "pytest -q tests && python run_all.py"]
