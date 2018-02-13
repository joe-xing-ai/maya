FROM continuumio/miniconda

ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/lib/

RUN apt-get update
RUN apt-get dist-upgrade -y
RUN apt-get install -y python3 python3-dev python3-pip

ADD artifacts/libnanomsg.so.5.1.0 /usr/local/lib/libnanomsg.so
ADD artifacts/nanomsg /usr/local/include/nanomsg
COPY . maya
WORKDIR /maya
RUN conda env create -f environment.yml
RUN apt-get install --yes emacs
CMD /bin/bash -c  "source activate maya_public \
    && python test_maya.py --batch_mode --maya_path artifacts/car_race.x86_64"