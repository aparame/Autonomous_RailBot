'use strict';

const express = require('express');
const app = express();
app.use(express.static('dist'))



const PORT = process.env.PORT || 4040;
app.listen(PORT, () => {
  console.log(`App listening on port ${PORT}`);
  console.log('Press Ctrl+C to quit.');
});