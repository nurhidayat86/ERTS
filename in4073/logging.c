bool write_log(struct log_t log_write, uint8_t index) {
	return flash_write_bytes(index*sizeof(struct log_t), (uint8_t *) &log_write, sizeof(struct log_t));
}

bool download_log (struct log_t *log_read) {
	uint16_t index=0;
	bool status = true;
	while (((index*sizeof(log_t)) <= 125000) && (status == true))
	{
		status = flash_read_bytes(index*sizeof(struct log_t), (uint8_t *) &log_read, sizeof(struct log_t));
		if (status == true)
		{
			//send encoded data to the pc here here
		}
		else
		{
			status = false;
			break;
		}
	}
	return status;
}