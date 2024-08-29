import logging
from state import main


def configure_logger():
    logging.basicConfig(
        level=logging.DEBUG,  # Set to DEBUG to capture all levels of log messages
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',  # Log format
        handlers=[
            logging.StreamHandler(),  # Output to console
            logging.FileHandler('pick-plaz.log', mode='a')  # Save to a file
        ]
    )


if __name__ == "__main__":
    configure_logger()
    main(mock=True)