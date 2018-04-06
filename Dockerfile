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

RUN conda update -n base -y conda

RUN cd /tmp && \
    git clone https://github.com/udacity/FCND-Term1-Starter-Kit.git
RUN cd /tmp/FCND-Term1-Starter-Kit && \
    conda env create -f environment.yml && \
    conda clean -tp

RUN conda config --add channels conda-forge
RUN conda update -n base -y conda && \
    conda install -n fcnd -y \
        jupyterthemes \
        msgpack-python \
        networkx==2.1 \
        pandas \
        scikit-image \
        scikit-learn \
        scipy \
        shapely \
        tqdm


RUN echo "source activate fcnd" >> ~/.bashrc
RUN echo "jt -t monokai -T -N" >> ~/.bashrc
RUN echo "jupyter notebook --no-browser --allow-root --ip=0.0.0.0 &> /dev/null &" >> ~/.bashrc

WORKDIR /app
COPY . /app/

# Define default command.
CMD ["bash"]

EXPOSE 12345
