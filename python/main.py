import logging

if __name__ == "__main__":
    from state import main
    main()

    logging.basicConfig(
        level=logging.DEBUG,  # Set to DEBUG to capture all levels of log messages
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',  # Log format
        handlers=[
            logging.StreamHandler(),  # Output to console
            logging.FileHandler('pick-plaz.log', mode='w')  # Save to a file
        ]
    )