#!/bin/bash

cd habitat-sim/

conda create -n habitat python=3.6 cmake=3.14.0
conda activate habitat
pip install -r requirements.txt

PYTHON_VERSION="$( python -c 'import sys; print(".".join(map(str, sys.version_info[:2])))' )"
PIL_VERSION="$(python -c 'import PIL; print(PIL.__version__)')"
NUMPY_VERSION="$(python -c 'import numpy as np; print(np.__version__)')"

cd ../habitat-lab/

pip install -r ./requirements.txt
reqs=(./habitat_baselines/**/requirements.txt)
pip install "${reqs[@]/#/-r}"

python setup.py develop --all
pip install . #Reinstall to trigger sys.path update
cd ../habitat-sim/

conda install -c aihabitat-nightly -c conda-forge habitat-sim  "python=${PYTHON_VERSION}" "numpy==${NUMPY_VERSION}" "pillow==${PIL_VERSION}"

# These needed to be additionally installed for me
pip install pycryptodomex
pip install gnupg
pip install rospkg

mkdir output/

cd ..

cd habitat_interface/
