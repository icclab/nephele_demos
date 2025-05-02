FROM nepheleproject/vo-wot:latest

RUN pip install -r /vo-wot/examples/plenary-demo/app-1/requirements.txt --no-cache-dir
ENV PYTHONBUFFERED 1

WORKDIR /vo-wot/examples/plenary-demo/app-1

CMD [ "python3", "-m" , "flask", "run", "--host=0.0.0.0"]
