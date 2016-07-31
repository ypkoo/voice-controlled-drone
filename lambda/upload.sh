#! /bin/bash

FUNCTION="YourFunctionName"
PACKAGE="$FUNCTION.zip"
FILEPATH="fileb://`pwd`/${PACKAGE}"

rm -rf ${PACKAGE}
zip -r ${PACKAGE} index.py paho awsCerts

aws lambda update-function-configuration --function-name "$FUNCTION" --handler "index.lambda_handler"
aws lambda update-function-code    --function-name "$FUNCTION"    --zip-file "${FILEPATH}"
