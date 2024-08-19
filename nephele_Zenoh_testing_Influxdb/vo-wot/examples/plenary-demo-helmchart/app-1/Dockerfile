FROM nepheleproject/vo-wot

RUN pip install -r examples/plenary-demo/app-1/requirements.txt
ENV PYTHONBUFFERED 1

WORKDIR examples/plenary-demo/app-1

CMD [ "python3", "-m" , "flask", "run", "--host=0.0.0.0"]
