package data

import (
	"bytes"
	"fmt"
	"log"
	"os/exec"
)

func McapToJson(mcap []byte) ([]byte, error) {
	log.Println("Converting MCAP to JSON")
	return pythonConverter(mcap)
}

// Temp solution for converting MCAP to JSON, as the current MCAP support for GO is a bit lacking
func pythonConverter(mcap []byte) ([]byte, error) {
	pythonConverter := exec.Command("python3", "./mcap_to_json.py", string(mcap))

	stdout := new(bytes.Buffer)
	stderr := new(bytes.Buffer)
	pythonConverter.Stdout = stdout
	pythonConverter.Stderr = stderr

	if err := pythonConverter.Run(); err != nil {
		return nil, err
	}

	if len(stderr.Bytes()) > 0 {
		return nil, fmt.Errorf("failed to convert mcap to json: %s", string(stderr.Bytes()))
	}

	return stdout.Bytes(), nil
}
