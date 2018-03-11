FROM continuumio/anaconda3

RUN apt-get update && apt-get install -y \
        build-essential \
        cmake \
        git \
        htop \
        tree \
        unzip \
        vim \
        wget

RUN conda update -n base conda

RUN cd /tmp && \
    git clone https://github.com/udacity/FCND-Term1-Starter-Kit.git
RUN cd /tmp/FCND-Term1-Starter-Kit && \
    conda env create -f environment.yml && \
    conda clean -tp

RUN conda install -n fcnd scipy -y
RUN conda install -n fcnd scikit-image -y

WORKDIR /app
COPY . /app/

# Define default command.
CMD ["bash"]

EXPOSE 12345
