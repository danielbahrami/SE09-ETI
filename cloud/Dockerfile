FROM golang:1.23.1

WORKDIR /app

# Just used for the temp solution for mcap to json
RUN apt-get update
RUN apt-get install python3.6
# install python3
# install python dependencies

COPY . ./
RUN go mod download

RUN CGO_ENABLED=0 GOOS=linux go build .

CMD ["./cloud"]
