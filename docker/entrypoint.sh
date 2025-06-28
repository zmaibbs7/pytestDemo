#!/usr/bin/env bash
set -e
pip install pytest-play

pytest -q \
  --yd yaml_test \
  --env ci \
  --file "$TEST_FILE" \
  --junitxml  /results/junit-${TEST_FILE%.yaml}.xml \
  --alluredir /results/allure-${TEST_FILE%.yaml}
