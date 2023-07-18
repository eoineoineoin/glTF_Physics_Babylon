const path = require("path");
const fs = require("fs");
const HtmlWebpackPlugin = require("html-webpack-plugin");
const appDirectory = fs.realpathSync(process.cwd());

module.exports = {
  entry: path.resolve(appDirectory, "src/app.ts"), //path to the main .ts file
  output: {
    filename: "js/bundleName.js", //name for the js file that is created/compiled in memory
    clean: true,
    libraryTarget: "umd",
  },
  resolve: {
    extensions: [".tsx", ".ts", ".js"],
    fallback: { fs: false, path: false },
    alias: {
      core: path.resolve(
        __dirname,
        "../Babylon.js/packages/dev/core/dist"
      ),
      loaders: path.resolve(
        __dirname,
        "../Babylon.js/packages/dev/loaders/dist"
      ),
     },
  },
  devServer: {
    host: "0.0.0.0",
    port: 9000, //port that we're using for local host (localhost:9000)
    static: path.resolve(appDirectory, "public"), //tells webpack to serve from the public folder
    hot: true,
    devMiddleware: {
      publicPath: "/",
    },
  },
  module: {
    rules: [
      {
        test: /\.js$/i,
        sideEffects: true,
      },
      {
        test: /\.tsx?$/,
        use: "ts-loader",
        exclude: /node_modules/,
      },
      {
        test: /\.wasm$/i,
        type: "asset/resource",
      },
    ],
  },
  plugins: [
    new HtmlWebpackPlugin({
      inject: true,
      template: path.resolve(appDirectory, "public/index.html"),
    }),
  ],
  mode: "development",
};
