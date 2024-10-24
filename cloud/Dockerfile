FROM golang:1.23.1

WORKDIR /app

COPY . ./
RUN go mod download

RUN CGO_ENABLED=0 GOOS=linux go build .

CMD ["./cloud"]