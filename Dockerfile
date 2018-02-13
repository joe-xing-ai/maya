FROM continuumio/miniconda3

ENV DEBIAN_FRONTEND=noninteractive


RUN apt-get update
RUN apt-get dist-upgrade -y
RUN apt-get install -y python3 python3-dev python3-pip

ADD libnanomsg.so.5.1.0 /usr/local/lib/libnanomsg.so
COPY . maya
WORKDIR /maya
RUN pip3 install -U pip
RUN conda env create -f environment.yml
RUN source activate maya_public