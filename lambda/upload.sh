#! /bin/bash

# ROLE="arn:aws:iam::520179866408:role/lambda_basic_execution"	# :function:DroneControl
# HANDLER="index.handler"

# MEMORY=128
# TIMEOUT=10

#rm -rf node_modules/
#npm install

#FUNCTION=`basename \`pwd\``
FUNCTION="KooDrone"
PACKAGE="$FUNCTION.zip"
FILEPATH="fileb://`pwd`/${PACKAGE}"

rm -rf ${PACKAGE}
#zip -r ${PACKAGE} *.js node_modules @ --exclude=*aws-sdk*
zip -r ${PACKAGE} index.py paho awsCerts

#aws lambda delete-function --function-name "$FUNCTION"

# aws lambda create-function \
#        --function-name "$FUNCTION" \
#        --zip-file "${FILEPATH}" \
#        --role "$ROLE" \
#        --handler "${HANDLER}" \
# 		--region us-east-1 \
#        --runtime nodejs \
#		--timeout 10

aws lambda update-function-configuration --function-name "$FUNCTION" --handler "index.lambda_handler"
aws lambda update-function-code    --function-name "$FUNCTION"    --zip-file "${FILEPATH}"
