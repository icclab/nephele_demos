FROM nginx

WORKDIR /etc/nginx
COPY ./nginx.conf ./nginx.conf
COPY ./default.conf ./conf.d/default.conf
EXPOSE 80
ENTRYPOINT [ "nginx" ]
CMD [ "-g", "daemon off;" ]
