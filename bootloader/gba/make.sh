#!/bin/bash
docker run --rm -it -v $(pwd):/mybuild devkitpro/devkitarm bash -c "cd /mybuild; make $@"
