Build the container with the following command:

``
docker build -t myflaskapp .
``

Then you can run it locally to test it with:

``
docker run -p 5000:5000 myflaskapp
``
