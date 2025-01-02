package data

import (
	"bytes"
	"fmt"
	"log"
	"os"
	"os/exec"

	"github.com/google/uuid"
)

func McapToJson(mcap []byte) ([]byte, error) {
	log.Println("Converting MCAP to JSON")
	return pythonConverter(mcap)
}

func pythonConverter(mcap []byte) ([]byte, error) {
	id := uuid.New().String()
	file, err := os.Create(id + ".mcap")
	defer os.Remove(id + ".mcap")
	if err != nil {
		return nil, err
	}
	if _, err = file.Write(mcap); err != nil {
		return nil, err
	}
	pythonConverter := exec.Command("python3", "./mcap_to_json.py", id+".mcap")

	stdout := new(bytes.Buffer)
	stderr := new(bytes.Buffer)
	pythonConverter.Stdout = stdout
	pythonConverter.Stderr = stderr

	if err := pythonConverter.Run(); err != nil {
		return nil, fmt.Errorf("python script failed: %w", err)
	}

	if len(stderr.Bytes()) > 0 {
		return nil, fmt.Errorf("failed to convert mcap to json: %s", string(stderr.Bytes()))
	}

	return stdout.Bytes(), nil
}
