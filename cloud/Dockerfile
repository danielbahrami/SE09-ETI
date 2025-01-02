FROM golang:1.23.1

WORKDIR /app

RUN apt-get update
RUN apt-get install python3.6
RUN apt install python3-pip -y
RUN pip3 install mcap --break-system-packages

COPY . ./
RUN go mod download

RUN CGO_ENABLED=0 GOOS=linux go build .

CMD ["./cloud"]
