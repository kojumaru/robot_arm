FROM python:3.11-slim

# システムパッケージのインストール
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    libusb-1.0-0-dev \
    liblapack-dev \
    libblas-dev \
    gfortran \
    && rm -rf /var/lib/apt/lists/*

# 作業ディレクトリ設定
WORKDIR /app

# requirements.txt をコピー（更新版）
COPY requirements.txt .

# Cython をインストール（ur_ikfast のコンパイルに必須）
RUN pip install --no-cache-dir Cython

# Python パッケージをインストール
RUN pip install --no-cache-dir -r requirements.txt

# ur_ikfast を git からクローンしてインストール
RUN git clone https://github.com/cambel/ur_ikfast.git /tmp/ur_ikfast && \
    cd /tmp/ur_ikfast && \
    git checkout 85b8274030113b52ab36129da8c0f6e8f971dbd7 && \
    pip install -e . && \
    rm -rf /tmp/ur_ikfast

# ソースコードをコピー
COPY . .

# SpaceMouse の USB アクセスを許可（--privileged で実行時に有効）
CMD ["python", "ur3e_ik.py"]
