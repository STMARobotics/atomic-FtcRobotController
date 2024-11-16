package org.firstinspires.ftc.teamcode;

import fi.iki.elonen.NanoHTTPD;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;

public class LimelightWebProxy extends NanoHTTPD {

    private static final String LIMELIGHT_IP = "http://localhost:5801"; // Replace with your Limelight's IP address

    // Constructor for the server listening on port 5000
    public LimelightWebProxy(int port) {
        super(port); // Pass the port number to the superclass constructor
    }

    // Handle incoming requests
    @Override
    public Response serve(IHTTPSession session) {
        String responseText = "Limelight Web Proxy Running!";

        try {
            // Build the URL to access Limelight's web interface
            URL url = new URL(LIMELIGHT_IP + session.getUri()); // Forward request to Limelight's IP
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");

            // Get the response from Limelight's web interface
            try (InputStream inputStream = connection.getInputStream()) {
                // Manually read the bytes if 'readAllBytes' is unavailable
                ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
                int byteRead;
                while ((byteRead = inputStream.read()) != -1) {
                    byteArrayOutputStream.write(byteRead);
                }

                responseText = new String(byteArrayOutputStream.toByteArray(), StandardCharsets.UTF_8);
            }

        } catch (IOException e) {
            responseText = "Error accessing Limelight: " + e.getMessage(); // Return error if unable to access Limelight
        }

        // Return the content (HTML) or error message to the client
        return newFixedLengthResponse(responseText);
    }

    // Main method to start the server
    public static void main(String[] args) {
        try {
            new LimelightWebProxy(5000).start(); // Start the server on port 5000
            System.out.println("Server started on port 5000");
        } catch (IOException e) {
            System.err.println("Could not start server: " + e.getMessage());
        }
    }
}
