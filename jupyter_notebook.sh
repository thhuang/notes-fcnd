#!/usr/bin/env bash

pip install --upgrade jupyterthemes
jt -t monokai -T -N
jupyter notebook --no-browser --allow-root --ip=0.0.0.0 &> /dev/null &