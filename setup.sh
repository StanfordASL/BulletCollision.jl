#!/bin/bash

source ./install-bullet3.sh
source ./activate.sh

julia-0.6 -e "Pkg.init()"
ln -s -f ../../REQUIRE packages/v0.6/REQUIRE
julia-0.6 -e "Pkg.resolve()"
