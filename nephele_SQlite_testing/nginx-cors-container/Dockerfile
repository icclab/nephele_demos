FROM nginx

WORKDIR /etc/nginx
COPY ./default.conf ./conf.d/default.conf
COPY ./nginx.conf ./nginx.conf
#COPY ./race_1.bag /usr/share/nginx/html/race_1.bag
EXPOSE 80
ENTRYPOINT [ "nginx" ]
CMD [ "-g", "daemon off;" ]
